#!/bin/python3

from frobs_rl.common import ros_gazebo
from sensor_msgs.msg import JointState

import numpy as np
import sys
import moveit_commander
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import rospy, sys, numpy as np
import cv2, cv_bridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import tf2_ros
from tf2_ros import Buffer
from tf2_ros import TransformListener
# from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import PoseStamped


class ur5_vision:
    def __init__(self):
        rospy.init_node("ur5_vision", anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/rx200/camera1/rgb/image_raw', Image, self.image_callback)

        self.point_cloud_sub = rospy.Subscriber('/rx200/camera1/depth/points', PointCloud2,
                                                self.point_cloud_callback)

        self.cube_pose = Pose()
        # self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)
        self.cube_point = Point()

        # to find the pose of the cube with respect to the robot
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer)
        self.from_frame = "camera_depth_optical_frame"
        self.to_frame = "rx200/base_link"
        # self.to_frame = "world"

    def point_cloud_callback(self, point_cloud):

        # center of the cube
        center_x = self.cx
        center_y = self.cy

        box_position_wrt_camera_frame = self.pixel_to_3d_point(point_cloud, center_x, center_y)

        # box_position_wrt_base_frame = self.transform_between_frames(box_position_wrt_camera_frame, self.from_frame,
        #                                                             self.to_frame)

        box_position_wrt_base_frame = self.transform_between_frames(box_position_wrt_camera_frame, self.from_frame,
                                                                    self.to_frame)

        print(f"box_position_wrt_base_frame:\n{box_position_wrt_base_frame}")

    def pixel_to_3d_point(self, point_cloud, center_x, center_y):

        # 2D structure of the point cloud. If the cloud is unordered, height is
        # 1 and width is the length of the point cloud.
        # width = point_cloud.width
        # height = point_cloud.hight

        # Convert from center_x (column or width), center_y(row or height) to position in array
        # where X, Y, Z data starts
        # point_step  # Length of a point in bytes
        # row_step  # Length of a row in bytes
        arrayPosition = (center_x * point_cloud.point_step) + (center_y * point_cloud.row_step)

        # compute position in array where x,y,z data start
        arrayPosX = arrayPosition + point_cloud.fields[0].offset  # X has an offset of 0
        arrayPosY = arrayPosition + point_cloud.fields[1].offset  # Y has an offset of 4
        arrayPosZ = arrayPosition + point_cloud.fields[2].offset  # Z has an offset of 8

        # I think this is with reference to the camera frame. we need it in the world frame
        x = float(point_cloud.data[arrayPosX])
        y = float(point_cloud.data[arrayPosY])
        z = float(point_cloud.data[arrayPosZ])

        # box_position_camera_frame = self.cube_point

        box_position_camera_frame = Point()

        box_position_camera_frame.x = x
        box_position_camera_frame.y = y
        box_position_camera_frame.z = z

        print(f"cube Pose with respect to the camera frame:\n{box_position_camera_frame}")

        return box_position_camera_frame

    def transform_between_frames(self, input_pose, input_frame, output_frame):

        input_pose_stamped = PoseStamped()
        input_pose_stamped.pose.position = input_pose
        input_pose_stamped.header.frame_id = input_frame
        input_pose_stamped.header.stamp = rospy.Time.now()

        output_pose = self.tf_buffer.transform(input_pose_stamped, output_frame, rospy.Duration(1))

        return output_pose.pose.position

    def colour_centroid(self, image):

        # END BRIDGE
        # BEGIN HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # END HSV
        # BEGIN FILTER
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        # todo: I changed the following so it works with opencv4
        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # area = cv2.contourArea(cnts)

        # this is to get the size of our output of the kinect
        h, w, d = image.shape
        # print("H:", h, "W:", w, "D:", d) #
        # H: 480 W: 640 D: 3

        # BEGIN FINDER
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cz = int(M['m02'] / M['m00'])  # i#m not sure about this. got from chatgpt

            # cx range (55,750) cy range( 55, ~ )
            # END FINDER
            # Isolate largest contour
            #  contour_sizes = [(cv2.contourArea(contour), contour) for contour in cnts]
            #  biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
            for i, c in enumerate(cnts):
                area = cv2.contourArea(c)
                if area < 7500:
                    self.track_flag = True
                    self.cx = cx
                    self.cy = cy
                    self.cz = cz
                    self.error_x = self.cx - w / 2
                    self.error_y = self.cy - (h / 2 + 195)
                    self.cube_pose.position.x = cx
                    self.cube_pose.position.y = cy
                    self.cube_pose.position.z = cz
                    # tracker.flag1 = self.track_flag
                    # tracker.error_x = self.error_x
                    # tracker.error_y = self.error_y

                    # (_,_,w_b,h_b)=cv2.boundingRect(c)
                    # print w_b,h_b
                    # BEGIN circle
                    cv2.circle(image, (cx, cy), 10, (0, 0, 0), -1)
                    cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx - 5), int(cy + 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.drawContours(image, cnts, -1, (255, 255, 255), 1)
                    # BGIN CONTROL

                    # let's print the postion
                    print("Pixel Pose of the cube:\n", self.cube_pose.position)

                    break
                else:
                    self.track_flag = False
                    # tracker.flag1 = self.track_flag

        # self.cxy_pub.publish(tracker)
        cv2.namedWindow("window", 1)
        cv2.imshow("window", image)
        cv2.waitKey(1)


    def image_callback(self, msg):
        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.colour_centroid(image)



follower = ur5_vision()
rospy.spin()
