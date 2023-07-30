#!/bin/python3

import os
import sys
import numpy
import rospy
import actionlib
import rostopic
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)

from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import tf

# for the ur5e robot arm
arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
arm_up_joint_positions = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]
arm_home_joint_positions = [0.0, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]

# for the rg6 gripper
gripper_joint_names = ["finger_joint"]
gripper_fully_open = -1.0
gripper_fully_close = 1.0
gripper_partially_open = 0.0
gripper_partially_close = 0.5


def move_joints(self, joints_array, epsilon=0.05, update_rate=10, time_sleep=0.05, check_position=True):
    i = 0
    for publisher_object in self.publishers_array:
        joint_value = Float64()
        joint_value.data = joints_array[i]
        rospy.logdebug("JointsPos>>" + str(joint_value))
        publisher_object.publish(joint_value)
        i += 1

    if check_position:
        self.wait_time_for_execute_movement(joints_array, epsilon, update_rate)
    else:
        self.wait_time_movement_hard(time_sleep=time_sleep)


def wait_time_for_execute_movement(self, joints_array, epsilon, update_rate):
    rospy.logdebug("START wait_until_goal_achieved...")

    rate = rospy.Rate(update_rate)
    start_wait_time = rospy.get_rostime().to_sec()
    end_wait_time = 0.0

    rospy.logdebug("Desired JointsState>>" + str(joints_array))
    rospy.logdebug("epsilon>>" + str(epsilon))

    while not rospy.is_shutdown():
        current_joint_states = self._check_joint_states_ready()

        values_to_check = [current_joint_states.position[0],
                           current_joint_states.position[1],
                           current_joint_states.position[2]]

        vel_values_are_close = self.check_array_similar(joints_array, values_to_check, epsilon)

        if vel_values_are_close:
            rospy.logdebug("Reached JointStates!")
            end_wait_time = rospy.get_rostime().to_sec()
            break
        rospy.logdebug("Not there yet, keep waiting...")
        rate.sleep()
    delta_time = end_wait_time - start_wait_time
    rospy.logdebug("[Wait Time=" + str(delta_time) + "]")

    rospy.logdebug("END wait_until_jointstate_achieved...")

    return delta_time


def wait_time_movement_hard(self, time_sleep):
    rospy.logdebug("Test Wait=" + str(time_sleep))
    rospy.sleep(time_sleep)  # Sleeps for 1 sec


def check_array_similar(self, ref_value_array, check_value_array, epsilon):
    rospy.logdebug("ref_value_array=" + str(ref_value_array))
    rospy.logdebug("check_value_array=" + str(check_value_array))
    return numpy.allclose(ref_value_array, check_value_array, atol=epsilon)


def move_gripper(joint_data=0.0, wait_duration=5.0):
    """
    Function to execute a roslaunch with args.

    Args:
        joint_data (float): Joint position [fully open:-1, fully close:1]
        wait_duration (float): Time to wait for the execution of the movement

    Returns:
        bool: True if the gripper movement is successful and False otherwise.
    """
    # Create a publisher object
    pub = rospy.Publisher('/gripper_controller/command', Float64, queue_size=10)

    # Set the rate at which to publish messages (in Hz)
    rate = rospy.Rate(1)

    # Create a Float64 message
    msg = Float64()
    msg.data = joint_data

    # to keep track if we published to the topic. We are only going to publish once
    done = False

    # to keep track of the time, so we can stop incase if it's taking too much time to execute
    start_wait_time = rospy.get_rostime().to_sec()
    end_wait_time = 0.0
    # print("start_wait_time", start_wait_time)

    # we will give it only the specified amount of seconds maximum to execute the task
    while (end_wait_time - start_wait_time) != wait_duration and not done:
        connections = pub.get_num_connections()

        if connections > 0:
            # Publish the message
            pub.publish(msg)
            done = True
        else:
            rate.sleep()

        # we need to update the time
        end_wait_time = rospy.get_rostime().to_sec()
        # print("end_wait_time", end_wait_time)

    if not done:
        rospy.loginfo("Gripper movement is unsuccessful!")
        return False
    if done:
        rospy.loginfo("Gripper movement is successful!")
        return True


# we probably need to pause before getting the data,
# or we need to wait until it finish moving
def get_pos_rpy_frame(ref_frame="world", req_frame="wrist_2_link"):
    # Create a tf listener
    listener = tf.TransformListener()

    # Wait for the transform to be available
    listener.waitForTransform(ref_frame, req_frame, rospy.Time(), rospy.Duration(4.0))

    # Get the transform at the current time
    (trans, rot) = listener.lookupTransform(ref_frame, req_frame, rospy.Time(0))

    # Print the position and orientation
    print("Position:", trans)
    print("Orientation:", rot)

# Callback function to receive joint state messages
def joint_state_callback(data):
    # Extract the current joint angles
    joint_angles = data.position
    joint_velocities = data.velocity
    joint_efforts = data.effort

    # print(joint_angles)

if __name__ == "__main__":
    rospy.init_node("prepare_simulated_robot")

    rospy.loginfo("Waiting for arm_controller...")
    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.loginfo("Waiting for gripper_controller...")
    # gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    # gripper_client.wait_for_server()
    # gripper_publisher = rospy.Publisher("/gripper_controller/command", Float64, queue_size=10)
    # print("topic type:", rostopic.get_topic_type("/gripper_controller/command", blocking=True))
    rospy.loginfo("...connected.")

    # Subscribe to the joint states topic
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = [0.0] * len(arm_up_joint_positions)
    trajectory.points[0].velocities = [0.0] * len(arm_up_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(arm_up_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(1.0)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[1].positions = arm_home_joint_positions
    trajectory.points[1].velocities = [0.0] * len(arm_up_joint_positions)
    trajectory.points[1].accelerations = [0.0] * len(arm_up_joint_positions)
    trajectory.points[1].time_from_start = rospy.Duration(4.0)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[2].positions = arm_up_joint_positions
    trajectory.points[2].velocities = [0.0] * len(arm_up_joint_positions)
    trajectory.points[2].accelerations = [0.0] * len(arm_up_joint_positions)
    trajectory.points[2].time_from_start = rospy.Duration(7.5)

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)

    # gripper_goal = Float64()
    # gripper_goal.data = gripper_partially_close

    rospy.loginfo("Setting positions...")
    # get_pos_rpy_frame()
    # arm_client.send_goal(arm_goal)
    # gripper_publisher.publish(gripper_goal)
    # arm_client.wait_for_result(rospy.Duration(6.0))
    # gripper_publisher(1.0)
    get_pos_rpy_frame()
    # try:
    #     move_gripper(data=0.0, wait_duration=3)
    # except rospy.ROSInterruptException:
    #     print("error")

    # rate = rospy.Rate(1)
    # rate.sleep()
    # rospy.sleep(10)

    rospy.loginfo("...done")
