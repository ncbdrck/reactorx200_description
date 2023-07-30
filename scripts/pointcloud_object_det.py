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
from geometry_msgs.msg import Transform
from extended_object_detection.msg import SimpleObjectArray, BaseObject

from sensor_msgs.msg import PointCloud2
import pcl
import numpy as np


class extended_object_detection_class:
    def __init__(self):
        rospy.init_node("point_cloud_object_detection_pose", anonymous=False)


        # Create a subscriber for the point cloud topic
        point_cloud_sub = rospy.Subscriber("/head_mount_kinect2/depth/points", PointCloud2,
                                           self.red_cube_detection_callback)

        self.cube_point = Point()

        # to find the pose of the cube with respect to the robot
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer)
        # self.from_frame = "kinect2_kinect2_rgb_optical_frame"
        self.from_frame = "kinect2_kinect2_ir_optical_frame"
        self.to_frame = "rx200/base_link"
        # self.to_frame = "world"

    def transform_between_frames(self, input_pose, input_frame, output_frame):
        """
        returns a point
        """


        input_pose_stamped = PoseStamped()
        input_pose_stamped.pose.position = input_pose
        input_pose_stamped.header.frame_id = input_frame
        input_pose_stamped.header.stamp = rospy.Time.now()

        output_pose = self.tf_buffer.transform(input_pose_stamped, output_frame, rospy.Duration(1))

        return output_pose.pose.position



    def red_cube_detection_callback(self, msg):
        """
        Callback function for the red cube detection

        This function updates the self.cube_pose attribute with the position of the detected red cube with respect to the robot_base.

        Args:
            msg: a PointCloud2 message containing data from the Kinect camera
        """

        # Get the frame of reference of the camera
        frame_id = msg.header.frame_id

        # Convert the PointCloud2 message to a pcl PointXYZRGB object
        cloud = pcl.PointCloud_PointXYZRGB()
        cloud.from_list(list(point_cloud2.read_points(msg, skip_nans=True)))

        # Filter the point cloud to keep only the red points
        red_filter = cloud.make_passthrough_filter()
        red_filter.set_filter_field_name("r")
        red_filter.set_filter_limits(200, 255)
        cloud = red_filter.filter()

        # Cluster the red points to find the red cube
        tree = cloud.make_kdtree()
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.02)
        ec.set_MinClusterSize(100)
        ec.set_MaxClusterSize(25000)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()

        # Filter out clusters that are too small or too large to be the cube
        cube_width = 0.05  # 5 cm
        min_cluster_size = (cube_width / 2) ** 3
        max_cluster_size = (cube_width * 2) ** 3
        valid_clusters = [c for c in cluster_indices if min_cluster_size <= len(c) <= max_cluster_size]

        # Find the cluster with the largest number of points (this is likely to be the red cube)
        largest_cluster = max(valid_clusters, key=lambda x: len(x))

        # Get the centroid of the largest cluster (this is the position of the red cube)
        points = np.array([cloud[i] for i in largest_cluster])
        centroid = np.mean(points, axis=0)

        # Get the position of the detected red cube
        cube_position = Point()
        cube_position.x = centroid[0]
        cube_position.y = centroid[1]
        cube_position.z = centroid[2]






follower = extended_object_detection_class()
rospy.spin()
