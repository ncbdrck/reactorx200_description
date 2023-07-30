#!/bin/python3
import rostopic
from frobs_rl.common import ros_gazebo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
import sys
import moveit_commander
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetWorldProperties, GetModelState


class Moveit(object):

    def __init__(self, arm_name, gripper_name=None, robot_description=None, ns=None):
        rospy.logwarn("===== Initialing Moveit Commander")

        # --- Init MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)

        # --- Instantiate a RobotCommander object
        # self.robot = moveit_commander.RobotCommander(robot_description="rx200/robot_description")
        self.robot = moveit_commander.RobotCommander(robot_description="rx200/robot_description", ns="rx200")

        # --- Instantiate a PlanningSceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface(ns="rx200")

        # ---Instantiate a MoveGroupCommander object.
        self.robot_arm_name = arm_name
        self.robot_arm = moveit_commander.MoveGroupCommander(name=self.robot_arm_name,
                                                             robot_description="rx200/robot_description", ns="rx200")

        if self.robot_arm.has_end_effector_link():
            print("We have an End-effector link: ", self.robot_arm.get_end_effector_link())
        else:
            print("Robot doesn't have an End-effector link! ")
            print("Please add an End-effector in your Moveit configuration package!")

        if gripper_name is not None:
            self.gripper_name = gripper_name
            self.gripper = moveit_commander.MoveGroupCommander(name=self.gripper_name,
                                                               robot_description="rx200/robot_description", ns="rx200")
        else:
            print("No gripper is added!")

        rospy.logwarn("===== Initialing Done")

    """
    Helper functions
    """

    # helper fn
    def arm_execute_pose(self, pose):
        ros_gazebo.gazebo_unpause_physics()

        self.robot_arm.set_pose_target(pose)
        result = self.arm_execute_trajectory()
        self.robot_arm.clear_pose_targets()

        ros_gazebo.gazebo_pause_physics()
        return result

    # helper fn
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

    # helper fn
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

    # helper fn
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
        current_joint_vals = arm_joint_vals + gripper_joint_vals
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
        ee_target.orientation.w = 1.0

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


def model_state_callback(msg):
    print("names", msg.name)


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

    # check if moveit is running
    rospy.wait_for_service("/rx200/move_group/trajectory_execution/set_parameters")
    print(rostopic.get_topic_type("/rx200/planning_scene", blocking=True))
    print(rostopic.get_topic_type("/rx200/move_group/status", blocking=True))

    move_ur5e_object = Moveit(arm_name='interbotix_arm', gripper_name='interbotix_gripper')

    # print("robot pose: ", move_ur5e_object.get_robot_pose())
    # print("robot rpy: ", move_ur5e_object.get_robot_rpy())
    # print("robot reference frame: ", move_ur5e_object.robot_arm.get_pose_reference_frame())

    # let's pause
    ros_gazebo.gazebo_pause_physics()
    print("random joint values: ", move_ur5e_object.get_randomJointVals())

    """
    get_randomPose() - This need to pause and unpause 
    """
    pose_done = False
    while not pose_done:
        ros_gazebo.gazebo_unpause_physics()
        goal = move_ur5e_object.get_randomPose()
        print("random goal values: ", goal.pose.position)
        # print("goal dtype:", type(goal))
        ros_gazebo.gazebo_pause_physics()

        goal_pose = np.array([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])
        cube_pose = np.array([goal.pose.position.x, goal.pose.position.y, 0.0])

        # check the goal
        ros_gazebo.gazebo_unpause_physics()
        pose_done = move_ur5e_object.check_goal(cube_pose)
        print("check cube_pose: ", pose_done)
        ros_gazebo.gazebo_pause_physics()

        # lazy to remove the while loop
        # pose_done = True

    # cube remove
    object_name = "red_cube"
    object_pkg_name = "reactorx200_description"
    object_file_name = "block.sdf"
    object_file_folder = "/models/block"
    object_offset = 0.8

    # rospy.wait_for_service("/gazebo/get_model_state")
    # model_states_sub = rospy.Subscriber("/gazebo/get_model_state", ModelStates, model_state_callback)
    # Wait for the required services to become available
    rospy.wait_for_service('/gazebo/get_world_properties')
    rospy.wait_for_service('/gazebo/get_model_state')

    # Create the service proxies
    get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    # ros_gazebo.gazebo_unpause_physics()
    # delete the model
    ros_gazebo.gazebo_delete_model(object_name)
    # ros_gazebo.gazebo_pause_physics()

    spawn_done = False
    count = 5

    while not spawn_done and count != 0:

        # ros_gazebo.gazebo_unpause_physics()
        done = ros_gazebo.gazebo_spawn_sdf_pkg(pkg_name=object_pkg_name, file_name=object_file_name,
                                               file_folder=object_file_folder, model_name=object_name,
                                               pos_x=cube_pose[0], pos_y=cube_pose[1], pos_z=object_offset)

        print("spawn:", done)

        # we need to unpause before ropy.sleep
        ros_gazebo.gazebo_unpause_physics()
        rospy.sleep(1)

        # Get the names of all the models in the world
        world_properties = get_world_properties()
        model_names = world_properties.model_names

        # Check if the model you're interested in is in the world
        if object_name in model_names:
            # Get the state of the model
            model_state = get_model_state(model_name=object_name, relative_entity_name="rx200/base_link")
            # Print the model's state
            rospy.loginfo('Model state: {}'.format(model_state.pose.position))
            spawn_done = True
        else:
            rospy.loginfo('Model not found in Gazebo.')
            count += -1

    # let's try to do pick and place
    pick_pose = cube_pose

    # place goal
    place_pose_done = False
    while not place_pose_done:
        ros_gazebo.gazebo_unpause_physics()
        goal = move_ur5e_object.get_randomPose()
        print("random goal values: ", goal.pose.position)
        # print("goal dtype:", type(goal))
        ros_gazebo.gazebo_pause_physics()

        place_goal_pose = np.array([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])

        # check the goal
        ros_gazebo.gazebo_unpause_physics()
        place_pose_done = move_ur5e_object.check_goal(place_goal_pose)
        # print("check cube_pose: ", pose_done)
        ros_gazebo.gazebo_pause_physics()

    # move the robot to pick pose
    # pseo = np.array([model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z])
    # mvmnt = move_ur5e_object.set_trajectory_ee(place_goal_pose)
    # print("reach:", mvmnt)

    action = [0, 0, 0.6, 0.6, 0.51377735]
    print("Move arm with Joints:", move_ur5e_object.set_trajectory_joints(action))

    pose = [0.20671873, 0, 0.2]
    print("Move arm with POSE:", move_ur5e_object.set_trajectory_ee(pose))
