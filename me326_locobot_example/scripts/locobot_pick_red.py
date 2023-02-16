#!/usr/bin/env python3
'''
Written by: Saksham Consul, Date: 2/1/2023
'''

import sys
import rospy
import numpy as np
import scipy as sp
from scipy import linalg
import geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

from std_msgs.msg import Float64

class OrientCamera(object):
	"""docstring for OrientCamera"""
	def __init__(self, tilt_topic = "/locobot/tilt_controller/command", pan_topic = "/locobot/pan_controller/command"):		
		self.orient_pub = rospy.Publisher(tilt_topic, Float64, queue_size=1, latch=True)
		self.pan_pub = rospy.Publisher(pan_topic, Float64, queue_size=1, latch=True)

	def tilt_camera(self,angle=0.5):
		msg = Float64()
		msg.data = angle
		self.orient_pub.publish(msg)
		# print("cause orientation, msg: ", msg)

	def pan_camera(self,angle=0.5):
		msg = Float64()
		msg.data = angle
		self.pan_pub.publish(msg)

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
		
	def close_gripper(self):
		gripper_goal = self.gripper_move_group.get_current_joint_values()
		print("grippers",gripper_goal)
		gripper_goal[0] = 0.037
		gripper_goal[1] = -0.037
		self.gripper_move_group.go(gripper_goal, wait=True)

	def open_gripper(self):
		gripper_goal = self.gripper_move_group.get_current_joint_values()
		gripper_goal[0] = -0.037
		gripper_goal[1] = 0.037
		self.gripper_move_group.go(gripper_goal, wait=True)


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

	def move_gripper_down_to_grasp(self):
		pose_goal = geometry_msgs.msg.Pose()

		pose_goal.position.x = 0.5
		pose_goal.position.y = 0.0
		pose_goal.position.z = 0.03

		v = np.matrix([0,1,0]) #pitch about y-axis
		th = 10*np.pi/180. #pitch by 45deg
		#note that no rotation is th= 0 deg

		pose_goal.orientation.x = v.item(0)*np.sin(th/2)
		pose_goal.orientation.y = v.item(1)*np.sin(th/2)
		pose_goal.orientation.z = v.item(2)*np.sin(th/2)
		pose_goal.orientation.w = np.cos(th/2)

		self.arm_move_group.set_pose_target(pose_goal)
		# now we call the planner to compute and execute the plan
		plan = self.arm_move_group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		self.arm_move_group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		self.arm_move_group.clear_pose_targets()

class LocobotMoveAndTrack(object):
    """Class for operations for locobot control tracking a goal pose using proportional velocity
    The pose of the Locobot is defined by its (x,y) position 
    """
    def __init__(self):
        self.base_position = rospy.wait_for_message("/locobot/mobile_base/odom", Odometry).pose
        rospy.loginfo("Base Location: %s", self.base_position)
        self.target_pose = None
        # Publisher to visualize point P upon which the Locobot is Holonomic
        self.point_P_control_point_visual = rospy.Publisher("/locobot/mobile_base/control_point_P",Marker,queue_size=1) #this can then be visualized in RVIZ (ros visualization)
        # Publisher for target pose
        self.target_pose_publisher = rospy.Publisher("/locobot/mobile_base/target_pose", Pose, queue_size=10) # topic to publish to update target goal
         # Publisher to visualize target pose
        self.target_pose_visual = rospy.Publisher("/locobot/mobile_base/target_pose_visual",Marker, queue_size=1)
        # Publisher to communicate info to move Locobot
        self.mobile_base_vel_publisher = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=1) #this is the topic we will publish to in order to move the base
        #set targets for when a goal is reached: 
        self.target_reached_error = 0.15
        self.L = 0.1 #this is the distance of the point P (x,y) that will be controlled for position. The locobot base_link frame points forward in the positive x direction, the point P will be on the positive x-axis in the body-fixed frame of the robot mobile base
        rospy.loginfo("Successfully initialized LocobotMoveAndTrack!")

        self.reached_goal = False

    def pub_point_P_marker(self):
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
        marker.pose.position.x = self.L  # center of the sphere in base_link frame
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 1.0 #red
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.point_P_control_point_visual.publish(marker)

    def pub_target_point_marker(self):
        #this is putting the marker in the world frame (http://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29)
        marker = Marker()
        marker.header.frame_id = "locobot/odom" #this will be the world frame for the real robot
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.ARROW
        # Set the marker scale
        marker.scale.x = 0.3  # arrow length
        marker.scale.y = 0.1 #arrow width
        marker.scale.z = 0.1 #arrow height

        # Set the marker pose
        marker.pose.position.x = self.target_pose.position.x  # center of the sphere in base_link frame
        marker.pose.position.y = self.target_pose.position.y
        marker.pose.position.z = self.target_pose.position.z
        marker.pose.orientation.x = self.target_pose.orientation.x
        marker.pose.orientation.y = self.target_pose.orientation.y
        marker.pose.orientation.z = self.target_pose.orientation.z
        marker.pose.orientation.w = self.target_pose.orientation.w

        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 0.0 #red
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Publish the marker
        self.target_pose_visual.publish(marker)
    
    def target_pose_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
        # print(20*"*")
        self.target_pose = data.pose
        self.target_pose_publisher.publish(self.target_pose)
        self.pub_target_point_marker()

    def mobile_base_callback(self, data):
        """
        This function is the callback for the message that is publishing the pose of mobile base.
        Note: the locobot bases are non-holonomic, meaning that like a wheelchair, they cannot 
        instantaneously move side-to-side, so to control the (x,y) position of the base, we must control a point (we will call P) 
        that is in front or behind the non-holonomic line (the line that connects the center of the wheels). It turns out 
        that it is possible to control this point in any direction we choose, but we cannot simultaneosuly control the position (x,y) and the planar angle (theta).
        So the strategy will be to move to the desired (x,y) location, then rotate to the desired angle sequentially (in this script, we will focus on just the xy motion, and leave as an exercise the rotation to the student)

        Note: this message topic /locobot/mobile_base/odom is published at about 30Hz, and we are using it to trigger our control script (so we control at the 
        frequency we obtain the state information)
        """

        # Step 1: Calculate the point P location (distance L on the x-axis), and publish the marker so it can be seen in Rviz
        #first determine the relative angle of the mobile base in the world xy-plane, this angle is needed to determine where to put the point P
        #the rotation will be about the world/body z-axis, so we will only need the qw, and qz quaternion components. We can then use knoweldge of the 
        #relationship between quaternions and rotation matricies to see how we must rotate the Lx vector into the world (odom) frame and add it to the base position
        #to obtain the point P (for more info on quaterions, see a primer at the bottom of this page: https://arm.stanford.edu/resources/armlab-references)
        qw = data.pose.pose.orientation.w
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        R11 = qw**2 + qx**2 - qy**2 -qz**2
        R12 = 2*qx*qz - 2*qw*qz
        R21 = 2*qx*qz + 2*qw*qz
        R22 = qw**2 - qx**2 + qy**2 -qz**2

        point_P = Point()
        #NOTE: the following assumes that when at the origin, the baselink and odom/world frame are aligned, and the z-axis points up. If this is not true then there is not a simple rotation about the z-axis as shown below
        point_P.x = data.pose.pose.position.x + self.L*R11
        point_P.y = data.pose.pose.position.y + self.L*R21
        point_P.z = 0.1 #make it hover just above the ground (10cm)

        #publish the point P marker for visualization in Rviz:
        self.pub_point_P_marker()

        if self.target_pose is None:
            #rospy.loginfo("Target Pose has not been received")
            pass
        else:
            rospy.loginfo("Target Pose: %s", self.target_pose)
            # Step 2: Calculate the error between the target pose for position control (this will relate to the proportoinal gain matrix, the P in PID control)
            err_x = self.target_pose.position.x - point_P.x
            err_y = self.target_pose.position.y - point_P.y
            error_vect = np.matrix([[err_x],[err_y]]) #this is a column vector (2x1); equivalently, we could use the transpose operator (.T): np.matrix([err_x ,err_y]).T  
            err_magnitude = np.linalg.norm(error_vect)   

            if err_magnitude > self.target_reached_error:
                Kp_mat = 0.5*np.eye(2) #proportional gain matrix

                #We will deal with this later (once we reached the position (x,y) goal), but we can calculate the angular error now - again this assumes there is only planar rotation about the z-axis, and the odom/baselink frames when aligned have x,y in the plane and z pointing upwards
                Rotation_mat = np.matrix([[R11,R12],[R21,R22]])
                #We can use matrix logarithm (inverse or Rodrigues Formula and exponential map) to get an axis-angle representation:
                axis_angle_mat = sp.linalg.logm(Rotation_mat)
                
                # This is the angle error: how should frame Base move to go back to world frame?
                angle_error = axis_angle_mat[0,1] #access the first row, second column to get angular error (skew sym matrix of the rotation axis - here only z component, then magnitude is angle error between the current pose and the world/odom pose which we will return to both at points A and B) 
                
                # This Angle is selected because its the frame rotation angle, how does Base appear from world?
                current_angle = axis_angle_mat[1,0] #this is also the angle about the z-axis of the base
                # Kp_angle_err = 0.2 #gain for angular error (here a scalar because we are only rotating about the z-axis)
            

                # Step 3: now put it all together to calculate the control input (velocity) based on the position error
                point_p_error_signal = Kp_mat*error_vect
                #The following relates the desired motion of the point P and the commanded forward and angular velocity of the mobile base [v,w]
                non_holonomic_mat = np.matrix([[np.cos(current_angle), -self.L*np.sin(current_angle)],[np.sin(current_angle),self.L*np.cos(current_angle)]])
                #Now perform inversion to find the forward velocity and angular velcoity of the mobile base.
                control_input = np.linalg.inv(non_holonomic_mat)*point_p_error_signal #note: this matrix can always be inverted because the angle is L
                #now let's turn this into the message type and publish it to the robot:
                control_msg = Twist()
                control_msg.linear.x = float(control_input.item(0)) #extract these elements then cast them in float type
                control_msg.angular.z = float(control_input.item(1))
                #now publish the control output:
                self.mobile_base_vel_publisher.publish(control_msg)

            else:
                rospy.loginfo("Reached red cube")
                self.reached_goal = True
                control_msg = Twist()
                control_msg.linear.x = 0
                control_msg.angular.z  = 0
                self.mobile_base_vel_publisher.publish(control_msg)

    def go_to_pose(self,target_pose=None):
        """
        Input: Target_pose - ROS geometry_msgs.msg.Pose type
        To see what it looks like, put the following in the terminal: $ rosmsg show geometry_msgs/Pose
        """
        while not self.reached_goal:
            # rospy.loginfo("Going to Goal")
            #Step 1: Subscribe to target pose
            red_cube_sub = rospy.Subscriber("/optitrack/red_cube/pose", PoseStamped, self.target_pose_callback)

            #Step 2: Subscribe to pose of the base, and use 'Point P control' (point right in front of the non-holonomic base's line) to move to the position, then orient to the target orinetation once the position is reached

            mobile_base_sub = rospy.Subscriber("/locobot/mobile_base/odom", Odometry, self.mobile_base_callback) #this says: listen to the odom message, of type odometry, and send that to the callback function specified
            # rospy.spin() #This is ros python's way of 'always listening' for the subscriber messages, and when it 
        # else:
        #     red_cube_sub.shutdown()
        #     mobile_base_sub.shutdown()

def main():
    rospy.init_node('locobot_moveandtrack')
    track_obj = LocobotMoveAndTrack() #instantiate object of the class (to call and use the functions)

    moveit_commander.roscpp_initialize(sys.argv)
    move_arm_obj = MoveLocobotArm(moveit_commander=moveit_commander)
    move_arm_obj.display_moveit_info()

    # Point the camera toward the blocks
    camera_orient_obj = OrientCamera()
    rospy.loginfo("TIlt Camera Down")
    camera_orient_obj.tilt_camera()

    rospy.loginfo("Move arm down for the camera")
    move_arm_obj.move_arm_down_for_camera()
    
    rospy.loginfo("Moving to goal")
    track_obj.go_to_pose()
    
    rospy.loginfo("Move Gripper down to grasp")
    move_arm_obj.move_gripper_down_to_grasp()

if __name__ == '__main__':
    #this is where the script begins, calls the main function
    main()

