import sys
import rospy
import numpy as np
import scipy as sp
from scipy import linalg
import geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from tf import TransformListener
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import cv2
from std_msgs.msg import Float64
from tf import TransformListener



class MoveLocobotArm(object):
    """docstring for MoveLocobotArm"""
    def __init__(self,moveit_commander=None):
        self.moveit_commander = moveit_commander
        self.robot = self.moveit_commander.RobotCommander() #this needs to be launched in the namespace of the robot (in this example, this is done in the launch file using 'group')
        self.scene = self.moveit_commander.PlanningSceneInterface()
        self.gripper_group_name = "interbotix_gripper"
        self.gripper_move_group = self.moveit_commander.MoveGroupCommander(self.gripper_group_name)
        self.arm_group_name = "interbotix_arm" #interbotix_arm and interbotix_gripper (can see in Rviz)
        self.arm_move_group = self.moveit_commander.MoveGroupCommander(self.arm_group_name)
        self.display_trajectory_publisher = rospy.Publisher('/locobot/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.arm_move_group.get_planning_frame()
        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.arm_move_group.get_end_effector_link()
        self.jnt_names = self.arm_move_group.get_active_joints()
        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        #create transform object
        self.listener = TransformListener()
        self.red_marker_arm_frame_publisher = rospy.Publisher("/locobot/blocks/red_marker_arm_frame", Marker, queue_size=1) #this is the topic we will publish to in order to move the base
        self.pose_goal = geometry_msgs.msg.Pose()

    def display_moveit_info(self):
        # We can get the name of the reference frame for this robot:
        print("============ Planning frame: %s" % self.planning_frame)
        # We can also print the name of the end-effector link for this group:
        print("============ End effector link: %s" % self.eef_link)
        print("============ Armgroup joint names: %s" % self.jnt_names)
        # We can get a list of all the groups in the robot:
        print("============ Available Planning Groups:", self.robot.get_group_names())
        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("\n")

    def move_arm_down_for_camera(self):
        #start here
        joint_goal = self.arm_move_group.get_current_joint_values()
        #['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        joint_goal[0] = -0.1115207331248822 #waist
        joint_goal[1] = -0.5313552376357276 #shoulder
        joint_goal[2] = 1.058371284458718 #elbow
        joint_goal[3] = -0.05608022936825474 #forearm_roll
        joint_goal[4] = 0.9302728070281328 #wrist_angle
        joint_goal[5] = -0.14247350829385486 #wrist_rotate
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.arm_move_group.go(joint_goal, wait=True)

    def marker_arm(self):
        #this is very simple because we are just putting the point P in the base_link frame (it is static in this frame)
        marker = Marker()
        marker.header.frame_id = "locobot/base_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.SPHERE
        # Set the marker scale
        marker.scale.x = 0.1  # radius of the sphere
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # Set the marker pose
        marker.pose.position.x = self.pose_goal.position.x
        marker.pose.position.y = self.pose_goal.position.y
        marker.pose.position.z = self.pose_goal.position.z
        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 0.5 #red
        marker.color.g = 0.5
        marker.color.b = 0.0
        # Publish the marker
        self.red_marker_arm_frame_publisher.publish(marker)

    def move_gripper_down_to_grasp_callback(self, goal):
        try:
            self.pose_goal.position.x = goal.point.x
            self.pose_goal.position.y = goal.point.y
            self.pose_goal.position.z = goal.point.z
            self.pose_goal.orientation.x = goal.orientation.x
            self.pose_goal.orientation.y = goal.orientation.y
            self.pose_goal.orientation.z = goal.orientation.z
            self.pose_goal.orientation.w = goal.orientation.w
            # publish a marker to the goal
            self.marker_arm()
            self.arm_move_group.set_pose_target(self.pose_goal)
            # now we call the planner to compute and execute the plan
            self.arm_move_group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            self.arm_move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self.arm_move_group.clear_pose_targets()
            self.open_gripper()
            self.close_gripper()
            self.move_arm_down_for_camera()
        except Exception as e:
            print("Error Pick Read: ", e)

    def open_gripper(self):
        gripper_goal = self.gripper_move_group.get_named_target_values('Open')
        self.gripper_move_group.go(gripper_goal, wait=True)
        self.gripper_move_group.stop()

    def close_gripper(self):
        gripper_goal = self.gripper_move_group.get_named_target_values("Closed")
        self.gripper_move_group.go(gripper_goal, wait=True)
        self.gripper_move_group.stop()