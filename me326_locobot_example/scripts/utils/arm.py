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

    def move_arm_slightly_up(self):
        #start here
        joint_goal = self.arm_move_group.get_current_joint_values()
        #['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        # joint_goal[0] = -0.1115207331248822 #waist
        joint_goal[1]-= 1.57# 0.5313552376357276 #shoulder
        joint_goal[2] += 1.48#1.058371284458718 #elbow
        # joint_goal[3] = -0.05608022936825474 #forearm_roll
        joint_goal[4] -= 0.52#0.9302728070281328 #wrist_angle
        rospy.loginfo("joint_goal is: %s", joint_goal)
        # joint_goal[5] = -0.14247350829385486 #wrist_rotate
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.arm_move_group.go(joint_goal, wait=True)

    def rotate_wrist(self):
        #start here
        joint_goal = self.arm_move_group.get_current_joint_values()
        joint_goal[5] -= 1.57#4247350829385486 #wrist_rotate
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

    def move_gripper_down_to_drop_callback(self,goal):
        try:
            self.pose_goal.position.x = goal.pose.position.x
            self.pose_goal.position.y = goal.pose.position.y
            self.pose_goal.position.z = goal.pose.position.z
            self.pose_goal.orientation.x = goal.pose.orientation.x
            self.pose_goal.orientation.y = goal.pose.orientation.y
            self.pose_goal.orientation.z = goal.pose.orientation.z
            self.pose_goal.orientation.w = goal.pose.orientation.w

            self.marker_arm()
            rospy.loginfo("Setting pose target")
            self.arm_move_group.set_pose_target(self.pose_goal)
            # now we call the planner to compute and execute the plan
            rospy.loginfo("Planning to pose target")
            self.arm_move_group.plan()
            rospy.loginfo("Moving to pose target")
            self.arm_move_group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            rospy.loginfo("Stopping arm movement")
            self.arm_move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivaflent function for clear_joint_value_targets()
            rospy.loginfo("Opening gripper")
            self.open_gripper()
            rospy.loginfo("Clearing pose targets")
            self.arm_move_group.clear_pose_targets()
            self.move_arm_down_for_camera()
        except Exception as e:
            print("Error Drop Read: ", e)

    def move_gripper_down_to_grasp_callback(self, goal):
        try:
            self.pose_goal.position.x = goal.pose.position.x
            self.pose_goal.position.y = goal.pose.position.y
            self.pose_goal.position.z = goal.pose.position.z
            self.pose_goal.orientation.x = goal.pose.orientation.x
            self.pose_goal.orientation.y = goal.pose.orientation.y
            self.pose_goal.orientation.z = goal.pose.orientation.z
            self.pose_goal.orientation.w = goal.pose.orientation.w
            # v = np.matrix([0,1,0]) #pitch about y-axis
            # th = 10*np.pi/180. #pitch by 45deg
            #note that no rotation is th= 0 deg

            # self.pose_goal.orientation.x = 0#v.item(0)*np.sin(th/2)
            # self.pose_goal.orientation.y = 0#v.item(1)*np.sin(th/2)
            # self.pose_goal.orientation.z = 0#v.item(2)*np.sin(th/2)
            # self.pose_goal.orientation.w = 1 #np.cos(th/2)
            # publish a marker to the goal
            self.marker_arm()
            rospy.loginfo("Opening gripper")
            self.open_gripper()
            rospy.loginfo("Setting pose target")
            self.arm_move_group.set_pose_target(self.pose_goal)
            # now we call the planner to compute and execute the plan
            rospy.loginfo("Planning to pose target")
            self.arm_move_group.plan()
            rospy.loginfo("Moving to pose target")
            self.arm_move_group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            rospy.loginfo("Stopping arm movement")
            self.arm_move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivaflent function for clear_joint_value_targets()
            
            # rospy.sleep(1)
            rospy.loginfo("Closing gripper")
            self.close_gripper()
            # rospy.sleep(1)
            rospy.loginfo("Clearing pose targets")
            self.arm_move_group.clear_pose_targets()
            self.move_arm_slightly_up()
            # self.rotate_wrist()
            # self.move_arm_down_for_camera()
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