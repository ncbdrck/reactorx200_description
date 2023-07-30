#!/bin/python3

from frobs_rl.common import ros_gazebo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
import sys
import moveit_commander
import rospy


class Moveit(object):

    def __init__(self, arm_name, gripper_name=None):
        rospy.logwarn("===== Initialing Moveit Commander")

        # --- Init MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)

        # --- Instantiate a RobotCommander object
        # self.robot = moveit_commander.RobotCommander(robot_description="rx200/robot_description")
        self.robot = moveit_commander.RobotCommander()

        # --- Instantiate a PlanningSceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface()

        # ---Instantiate a MoveGroupCommander object.
        self.robot_arm_name = arm_name
        self.robot_arm = moveit_commander.MoveGroupCommander(self.robot_arm_name)

        if self.robot_arm.has_end_effector_link():
            print("We have an End-effector link: ", self.robot_arm.get_end_effector_link())
        else:
            print("Robot doesn't have an End-effector link! ")
            print("Please add an End-effector in your Moveit configuration package!")

        if gripper_name is not None:
            self.gripper_name = gripper_name
            self.gripper = moveit_commander.MoveGroupCommander(self.gripper_name)
        else:
            print("No gripper is added!")

        rospy.logwarn("===== Initialing Done")

    def arm_execute_pose(self, pose):
        ros_gazebo.gazebo_unpause_physics()

        self.robot_arm.set_pose_target(pose)
        result = self.arm_execute_trajectory()
        self.robot_arm.clear_pose_targets()

        ros_gazebo.gazebo_pause_physics()
        return result

    def arm_execute_joint_trajectory(self, joint_target_values):

        self.joint_goal = self.get_joint_angles_robot_arm()

        # print("len joint_target_values", len(joint_target_values))
        # print("len self.joint_goal ", len(self.joint_goal))

        if len(joint_target_values) == len(self.joint_goal):
            for i in range(len(joint_target_values)):
                self.joint_goal[i] = float(joint_target_values[i])

        else:
            print("The input don't match with the robot joints. Not within the required limits!")
            return False

        ros_gazebo.gazebo_unpause_physics()
        self.robot_arm.set_joint_value_target(self.joint_goal)
        result = self.arm_execute_trajectory()
        ros_gazebo.gazebo_pause_physics()

        return result

    def gripper_execute_joint_command(self, target_joint_values):

        self.gripper_joints = self.get_joint_angles_gripper()

        # print("len joint_target_values", len(target_joint_values))
        # print("len self.gripper_joints ", len(self.gripper_joints))

        if len(target_joint_values) == len(self.gripper_joints):
            for i in range(len(target_joint_values)):
                self.gripper_joints[i] = float(target_joint_values[i])

        else:
            print("The input don't match with the gripper joints. Not within the required limits!")
            return False

        ros_gazebo.gazebo_unpause_physics()

        # self.gripper.set_joint_value_target(self.gripper_joints)
        result = self.gripper.go(self.gripper_joints, wait=True)
        self.gripper.stop()

        ros_gazebo.gazebo_pause_physics()

        return result

    def arm_execute_trajectory(self):
        """
        Assuming that the trajecties has been set to the self objects appropriately
        Make a plan to the destination in Homogeneous Space(x,y,z,yaw,pitch,roll)
        and returns the result of execution
        """
        self.plan = self.robot_arm.plan()

        result = self.robot_arm.go(wait=True)
        # result = self.robot_arm.execute(self.plan, wait=True)

        self.robot_arm.stop()
        # self.group.clear_pose_targets()

        return result

    # helper fn
    def robot_pose(self):
        robot_pose = self.robot_arm.get_current_pose()
        # --- Get EE position (this is how it is used)
        # ee_pos_v = self.get_ee_pose()  # Get a geometry_msgs/PoseStamped msg
        # self.ee_pos = np.array([ee_pos_v.pose.position.x, ee_pos_v.pose.position.y, ee_pos_v.pose.position.z])
        return robot_pose

    # helper fn
    def robot_rpy(self):
        robot_rpy = self.robot_arm.get_current_rpy()
        return robot_rpy

    # helper fn
    def gripper_pose(self, gripper_link=""):
        gripper_pose = self.gripper.get_current_pose(end_effector_link=gripper_link)
        # gripper_pose = self.gripper.get_current_pose().pose
        # gripper_pose = self.gripper.get_current_pose()
        # we get all the details from this not just pose. So we need to add ".pose" like the above code
        return gripper_pose

    # helper fn
    def gripper_rpy(self, gripper_link=""):
        gripper_rpy = self.gripper.get_current_rpy(end_effector_link=gripper_link)
        return gripper_rpy

    # helper fn
    def joint_angles_arm(self):
        current_jt_vals = self.robot_arm.get_current_joint_values()
        # print(type(current_jt_vals))
        return current_jt_vals

    # helper fn
    def joint_angles_gripper(self):
        gripper_joint_vals = self.gripper.get_current_joint_values()
        return gripper_joint_vals

    # helper fn
    def joint_angles(self):
        arm_joint_vals = self.joint_angles_arm()
        gripper_joint_vals = self.joint_angles_gripper()
        current_joint_vals = [arm_joint_vals, gripper_joint_vals]
        return current_joint_vals.copy()

    """
        Main functions we can call from the object
    """
    def is_goal_reachable(self, goal):
        """
        Check if the goal is reachable
        * goal is the desired XYZ of the EE
        """

        if isinstance(goal, type(np.array([0]))):
            goal = goal.tolist()

        goal[0] = float(goal[0])
        goal[1] = float(goal[1])
        goal[2] = float(goal[2])

        self.robot_arm.set_position_target(goal)
        plan = self.robot_arm.plan()
        result = plan[0]
        self.robot_arm.clear_pose_targets()

        return result

    def set_trajectory_ee(self, action):
        """
        Sets the Pose of the EndEffector based on the action variable.
        The action variable contains the position and orientation of the EndEffector.
        """
        # Set up a trajectory message to publish.
        ee_target = Pose()

        # Standard orientation
        # ee_target.orientation.x = 0
        # ee_target.orientation.y = 0
        # ee_target.orientation.z = 0.707
        # ee_target.orientation.z = 0
        # ee_target.orientation.w = 0

        ee_target.position.x = action[0]
        ee_target.position.y = action[1]
        ee_target.position.z = action[2]

        result = self.arm_execute_pose(ee_target)
        return result

    def set_trajectory_joints(self, q_positions):
        """
        Set a joint position target for the arm joints.
        """
        result = self.arm_execute_joint_trajectory(q_positions)

        return result

    def set_gripper_joints(self, joint_positions):
        """
        Set a joint position target for the gripper joints.
        """
        result = self.gripper_execute_joint_command(joint_positions)

        return result

    def get_robot_pose(self):
        """
        Returns geometry_msgs/PoseStamped
        """
        ros_gazebo.gazebo_unpause_physics()
        robot_pose = self.robot_pose()
        ros_gazebo.gazebo_pause_physics()
        return robot_pose

    def get_robot_rpy(self):
        """
        Returns a list of 3 elements defining the [roll, pitch, yaw] of the end-effector.
        """
        ros_gazebo.gazebo_unpause_physics()
        robot_rpy = self.robot_rpy()
        ros_gazebo.gazebo_pause_physics()
        return robot_rpy

    def get_gripper_pose(self, link=""):
        """
        Returns geometry_msgs/PoseStamped
        """
        ros_gazebo.gazebo_unpause_physics()
        gripper_pose = self.gripper_pose(gripper_link=link)
        ros_gazebo.gazebo_pause_physics()
        return gripper_pose

    def get_gripper_rpy(self, link=""):
        """
        Returns a list of 3 elements defining the [roll, pitch, yaw] of the end-effector.
        """
        ros_gazebo.gazebo_unpause_physics()
        gripper_rpy = self.gripper_rpy(gripper_link=link)
        ros_gazebo.gazebo_pause_physics()
        return gripper_rpy

    def get_joint_angles_robot_arm(self):
        ros_gazebo.gazebo_unpause_physics()
        robot_joint_angles = self.joint_angles_arm()
        ros_gazebo.gazebo_pause_physics()
        return robot_joint_angles

    def get_joint_angles_gripper(self):
        ros_gazebo.gazebo_unpause_physics()
        gripper_joint_angles = self.joint_angles_gripper()
        ros_gazebo.gazebo_pause_physics()
        return gripper_joint_angles

    def get_joint_angles(self):
        ros_gazebo.gazebo_unpause_physics()
        joint_angles = self.joint_angles()
        ros_gazebo.gazebo_pause_physics()
        return joint_angles

    def check_goal(self, goal):
        """
        Check if the goal is reachable
        * goal is a list with 3 elements, XYZ positions of the EE
        """
        result = self.is_goal_reachable(goal)
        return result

    def get_randomJointVals(self):
        return self.robot_arm.get_random_joint_values()

    def get_randomPose(self):
        return self.robot_arm.get_random_pose()

    # todo
    def create_joints_dict(self, joints_positions):
        """
        Based on the Order of the positions, they will be assigned to its joint name
        names_in_order:
            joint_1
            joint_2
            joint_3
            joint_4
            joint_5
            joint_6
        """

        assert len(joints_positions) == len(self.joint_names), "Wrong number of joints, there should be " + str(
            len(self.joint_names))
        joints_dict = dict(zip(self.joint_names, joints_positions))

        return joints_dict

"""
    This is for testing the above class.
    We can use the above class mainly to replace the usual moveit section in the almost all the moveit related 
    "Robot Environments" in the multiros framework
"""
if __name__ == '__main__':
    rospy.init_node(anonymous=True, name="test")

    # let's unpause gazebo first
    ros_gazebo.gazebo_unpause_physics()
    # ros_gazebo.gazebo_reset_sim()
    ros_gazebo.gazebo_reset_world()

    move_ur5e_object = Moveit(arm_name='interbotix_arm', gripper_name='interbotix_gripper')

    print("robot pose: ", move_ur5e_object.get_robot_pose())
    print("robot rpy: ", move_ur5e_object.get_robot_rpy())
    print("robot reference frame: ", move_ur5e_object.robot_arm.get_pose_reference_frame())
    print("Joint names: ", move_ur5e_object.robot_arm.get_joints())
    print("Joint angles: ", move_ur5e_object.get_joint_angles())


    # Here I was testing with the ur5e robot with the rg6 gripper. So we need to consider the following links
    # print("gripper reference frame: ", move_ur5e_object.gripper.get_pose_reference_frame())
    print("EE link gripper: ", move_ur5e_object.gripper.get_end_effector_link())
    print("left finger pose: ", move_ur5e_object.get_gripper_pose(link="rx200/left_finger_link"))
    print("left finger rpy: ", move_ur5e_object.get_gripper_rpy(link="rx200/left_finger_link"))
    print("right finger pose: ", move_ur5e_object.get_gripper_pose(link="rx200/right_finger_link"))
    print("right finger rpy: ", move_ur5e_object.get_gripper_rpy(link="rx200/right_finger_link"))


    # works
    # action = [0, 0, 0.6, 0.6, 0.51377735]
    # print("Move arm with Joints:", move_ur5e_object.set_trajectory_joints(action))

    # pose = [0.20671873, 0, 0.2]
    # print("Move arm with POSE:", move_ur5e_object.set_trajectory_ee(pose))

    # gripper
    # gripper_qpos_open = [0.0360, -0.0360] # open
    # print("Open Gripper:", move_ur5e_object.set_gripper_joints(gripper_qpos_open))

    # gripper_qpos_close = [0.0180, -0.0180]
    # print("Close Gripper:", move_ur5e_object.set_gripper_joints(gripper_qpos_close))

    # let's try to pick an object
    gripper_qpos_open = [0.0360, -0.0360]  # open
    print("Open Gripper:", move_ur5e_object.set_gripper_joints(gripper_qpos_open))
    pose_object = [0.45, 0, 0.05]
    print("Move arm with POSE:", move_ur5e_object.set_trajectory_ee(pose_object))
    gripper_qpos_close = [0.0180, -0.0180]
    print("Close Gripper:", move_ur5e_object.set_gripper_joints(gripper_qpos_close))
    home = [0, 0, 0, 0, 0]
    print("Move arm with Joints:", move_ur5e_object.set_trajectory_joints(home))







