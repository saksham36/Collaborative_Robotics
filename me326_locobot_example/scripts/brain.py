#!/usr/bin/env python3

from enum import Enum
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from controllers.trajectory_controller import TrajectoryController
from controllers.heading_controller import HeadingController
from geometry_msgs.msg import Twist, Pose2D, PoseStamped, PointStamped
import tf
from collections import deque
import numpy as np
import cv2
from utils.utils import wrapToPi, StochOccupancyGrid2D
from utils.arm import MoveLocobotArm
from planner.astar import AStar, compute_smoothed_traj
from std_msgs.msg import Float64
import moveit_commander
import sys

class Mode(Enum):
    IDLE = 0
    INIT = 1
    ALIGN = 2
    GOAL_ALIGN = 3
    MOVE = 4
    PICK = 5
    DROP = 6
    PARK = 7


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

class Brain:
    def __init__(self):
        rospy.init_node("locobot_brain", anonymous=True)
        self.mode = None
        self.switch_mode(Mode.INIT)

        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = None
        self.y_g = None
        self.theta_g = None

        # camera goal orientation
        self.explore_theta = None

        self.th_init = 0.0
        self.robot_dims = (2, 2)

        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0, 0]
        self.map_probs = []
        self.occupancy = None

        self.dilated_occupancy = OccupancyGrid()
        self.map_metadata = None

        # plan parameters
        self.plan_resolution = 0.1
        self.plan_horizon = 2
        self.plan_fail = 0
        self.fail_threshold = 5

        # time when we started following the plan
        self.current_plan_start_time = rospy.get_rostime()
        self.current_plan_duration = 0
        self.plant_start = [0.0,0.0]

        # robot limits
        self.v_max = 0.22
        self.om_max = 0.4
        self.v_des = 0.12
        self.theta_start_thresh = 0.05
        self.start_pos_thresh = 0.15

        # threshold at which to stop moving
        self.at_thresh = 0.6
        self.at_thresh_theta = 0.05

        # trajectory smoothing
        self.spline_alpha = 0.01
        self.traj_dt = 0.1

        # trajectory tracking controller parameters
        self.kpx = 0.6
        self.kpy = 0.6 
        self.kdx = 1
        self.kdy = 1

        # heading controller parameters
        self.kp_th = 0.5

        # controllers
        self.traj_controller = TrajectoryController(self.kpx, self.kpy, self.kdx, self.kdy, self.v_max, self.om_max)
        self.heading_controller = HeadingController(self.kp_th, self.om_max)

        # publishers
        self.nav_planned_path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
        self.dilated_map_pub = rospy.Publisher("/dilated_map", OccupancyGrid, queue_size=10) 
        self.vel_publisher = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=1)
        self.trans_listener = tf.TransformListener()

        # subscribers
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/locobot/goal", Pose2D, self.goal_callback)

        # previous state
        self.max_len = 50
        self.x_prev = deque([], maxlen = self.max_len)
        self.y_prev = deque([], maxlen = self.max_len)
        self.theta_prev = deque([], maxlen = self.max_len)
        
        # Camera
        self.camera = OrientCamera()
        
        # Arm
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_arm_obj = MoveLocobotArm(moveit_commander=moveit_commander)
        self.move_arm_obj.display_moveit_info()
        self.move_arm_obj.move_arm_down_for_camera()

    def get_current_plan_time(self):
        t = (rospy.get_rostime() - self.current_plan_start_time).to_sec()
        return max(0.0, t)  # clip negative time to 0
    
        
    def snap_to_grid(self, x, y):
        x_index = self.dilated_occupancy.info.resolution * int(x/self.dilated_occupancy.info.resolution)
        y_index = self.dilated_occupancy.info.resolution * int(y/self.dilated_occupancy.info.resolution)
        return (x_index, y_index)
    
    def camera_explore(self, start=False):
        if start:
            self.camera.tilt_camera(0.5)
            rospy.loginfo('Starting to explore world')
            self.explore_theta = self.theta
            self.prev_theta = self.theta
            self.total = 0
 
        if self.total >= 2 * np.pi: # TODO: Delete is None condition. Purely for debug
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0.0
            self.vel_publisher.publish(cmd_vel)
            rospy.loginfo("Finished exploring world")
            self.switch_mode(Mode.IDLE)
            self.explore_theta = None
            self.total = 0
       
        else:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0.2
            self.vel_publisher.publish(cmd_vel)

        self.total += abs(abs(self.prev_theta) - abs(self.theta))
        self.prev_theta = self.theta
                

    def goal_callback(self, data):
        if self.mode not in [Mode.INIT, Mode.PICK, Mode.PARK, Mode.MOVE, Mode.GOAL_ALIGN]:  # TODO: Check
            self.x_g = data.x
            self.y_g = data.y
            self.theta_g = data.theta
            rospy.loginfo('Received Goal')
            self.replan()

    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_metadata = msg.info
        self.map_probs = msg.data

        self.dilate_map()
        self.dilated_map_pub.publish(self.dilated_occupancy)
        
        if len(self.map_probs) == len(msg.data) and np.isclose(self.map_probs, msg.data, atol=1e-5).all() and len(self.dilated_occupancy.data) > 0:
            return

        if self.x_g is not None:
            # if we have a goal to plan to, replan
            rospy.loginfo("Replanning because of new map")
            self.replan()  # new map, need to replan

    def dilate_map(self):
        mask = np.array(self.map_probs) < 0
        grid = np.array(self.map_probs).reshape(self.map_height, self.map_width)
        grid_positive = np.where(grid > 0, grid, 0).astype('uint8')
        kernel = np.ones((3,3),'uint8')
        dilated_map = cv2.dilate(grid_positive, kernel, iterations=self.robot_dims[0]).astype('int8').flatten() 
        dilated_map[mask] = -1
        self.dilated_occupancy.data = tuple(dilated_map)
        self.dilated_occupancy.info = self.map_metadata
        self.dilated_occupancy.header.frame_id = "locobot/odom"
        self.occupancy = StochOccupancyGrid2D(
            self.map_resolution,
            self.map_width,
            self.map_height,
            self.map_origin[0],
            self.map_origin[1],
            3,
            self.dilated_occupancy.data,
            0.7
        )

    def near_goal(self):
        return (
            np.linalg.norm(np.array([self.x - self.x_g, self.y - self.y_g]))
            < self.near_thresh
        )

    def at_goal(self):
        # print(20*'*')
        # rospy.loginfo("Position distance: %s", np.linalg.norm(np.array([self.x - self.x_g, self.y - self.y_g])))
        return (
            np.linalg.norm(np.array([self.x - self.x_g, self.y - self.y_g]))
            < self.at_thresh
        )
    
    def aligned_goal(self):
        print(20*'*')
        rospy.loginfo("Theta: %s", self.theta)
        rospy.loginfo("Theta goal: %s", self.theta_g)
        rospy.loginfo("Orientation distance: %s", abs(wrapToPi(self.theta - self.theta_g)))
        return abs(wrapToPi(self.theta - self.theta_g)) < self.at_thresh_theta

    def aligned(self):
        return (
            abs(wrapToPi(self.theta - self.th_init)) < self.theta_start_thresh
        )
        
    def close_to_plan_start(self):
        return (
            abs(self.x - self.plan_start[0]) < self.start_pos_thresh
            and abs(self.y - self.plan_start[1]) < self.start_pos_thresh
        )
        
    def switch_mode(self, new_mode):
        print(20* "*")
        print(20* "*")
        rospy.loginfo("Switching from %s -> %s", self.mode, new_mode)
        print(20* "*")
        print(20* "*")
        self.mode = new_mode

    def publish_planned_path(self, path, publisher):
        # Publish planned plan for visualization
        path_msg = Path()
        path_msg.header.frame_id = "locobot/odom"
        for state in path:
            pose_st = PoseStamped()
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.pose.orientation.w = 1
            pose_st.header.frame_id = "locobot/odom"
            path_msg.poses.append(pose_st)
        publisher.publish(path_msg)

    def publish_control(self):
        """
        Runs appropriate controller depending on the mode. Assumes all controllers
        are all properly set up / with the correct goals loaded
        """
        t = self.get_current_plan_time()

        if self.mode == Mode.MOVE:
            V, om = self.traj_controller.compute_control(
                self.x, self.y, self.theta, t
            )
        elif self.mode in [Mode.ALIGN, Mode.GOAL_ALIGN]:
            V, om = self.heading_controller.compute_control(
                self.x, self.y, self.theta, t
            )
        else:
            V = 0.0
            om = 0.0

        cmd_vel = Twist()
        cmd_vel.linear.x = V
        cmd_vel.angular.z = om
        self.vel_publisher.publish(cmd_vel)

    def replan(self,):
        # Make sure we have a map
        if not self.occupancy:
            rospy.loginfo("No map yet, waiting")
            self.switch_mode(Mode.IDLE)
            return
        
        # Attempt to plan a path
        state_min = self.snap_to_grid(-self.plan_horizon, -self.plan_horizon)
        state_max = self.snap_to_grid(self.plan_horizon, self.plan_horizon)
        x_init = self.snap_to_grid(self.x, self.y)
        self.plan_start = x_init
        x_goal = self.snap_to_grid(self.x_g, self.y_g)
        
        problem = AStar(
            state_min,
            state_max,
            x_init,
            x_goal,
            self.occupancy,
            self.dilated_occupancy.info.resolution
        )

        success = problem.solve()
        if not success:
            rospy.loginfo("Planning failed")
            self.plan_fail += 1
            return
        rospy.loginfo("Planning succeeded")

        planned_path = problem.path
        rospy.loginfo("Planned path length: %d", len(planned_path))
        # Check whether path is too short
        if len(planned_path) < 4:
            rospy.loginfo("Path too short")
            self.switch_mode(Mode.PICK)
            self.publish_control()
            
            return

        # Smooth and generate a trajectory
        traj_new, t_new = compute_smoothed_traj(
            planned_path, self.v_des, self.spline_alpha, self.traj_dt
        )

        # If currently tracking a trajectory, check whether new trajectory will take more time to follow
        if self.mode == Mode.MOVE:
            t_remaining_curr = (
                self.current_plan_duration - self.get_current_plan_time()
            )

            # Estimate duration of new trajectory
            th_init_new = traj_new[0, 2]
            th_err = wrapToPi(th_init_new - self.theta)
            t_init_align = abs(th_err / self.om_max)
            t_remaining_new = t_init_align + t_new[-1]

            if t_remaining_new > t_remaining_curr:
                self.publish_planned_path(traj_new, self.nav_planned_path_pub)
                return

        # Otherwise follow the new plan
        self.publish_planned_path(traj_new, self.nav_planned_path_pub)
        self.traj_controller.load_traj(t_new, traj_new)

        self.current_plan_start_time = rospy.get_rostime()
        self.current_plan_duration = t_new[-1]

        self.th_init = traj_new[0, 2]
        self.heading_controller.load_goal(self.th_init)

        if not self.aligned() and self.mode != Mode.ALIGN:
            rospy.loginfo("Not aligned with start direction")
            self.switch_mode(Mode.ALIGN)
            return

        rospy.loginfo("Ready to move")
        self.plan_fail = 0
        self.switch_mode(Mode.MOVE)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # try to get state information to update self.x, self.y, self.theta
            try:
                # Getting trans, rotation wrt odom
                (translation, rotation) = self.trans_listener.lookupTransform(
                    "/locobot/odom", "/locobot/base_link", rospy.Time(0)
                )
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ) as e:
                self.current_plan = []
                rospy.loginfo("Navigator: waiting for state info")
                rate.sleep()
                continue

            if self.mode == Mode.INIT:
                if self.explore_theta is None:
                    self.camera_explore(start=True)
                else:
                    self.camera_explore()
                rate.sleep()
                continue
                
            if self.mode in [Mode.ALIGN, Mode.MOVE]:
                self.x_prev.append(self.x)
                self.y_prev.append(self.y)
                self.theta_prev.append(self.theta)

            if self.mode == Mode.ALIGN:
                if self.aligned():
                    self.current_plan_start_time = rospy.get_rostime()
                    self.switch_mode(Mode.MOVE)

            # elif self.mode == Mode.GOAL_ALIGN:
            #     if self.aligned_goal():
            #         self.switch_mode(Mode.PICK)

            elif self.mode == Mode.MOVE:
                if self.at_goal(): 
                    # if self.aligned_goal():
                    self.switch_mode(Mode.PICK)
                    # else:
                    #     self.switch_mode(Mode.GOAL_ALIGN)
                elif not self.close_to_plan_start():
                    rospy.loginfo("Replanning because far from start")
                    self.replan()
                elif (
                    rospy.get_rostime() - self.current_plan_start_time
                ).to_sec() > self.current_plan_duration:
                    rospy.loginfo("Replanning because out of time")
                    self.replan()  # we aren't near the goal but we thought we should have been, so replan

            elif self.mode == Mode.PICK:
                self.trans_listener.waitForTransform("/locobot/odom", "/locobot/base_link", rospy.Time(0), rospy.Duration(1.0))
                aux=PointStamped()
                # aux is an auxiliary point in the odom frame                
                aux.header.frame_id = "/locobot/odom"
                aux.header.stamp =rospy.Time(0)
                aux.point.x=self.x_g
                aux.point.y=self.y_g
                aux.point.z=0.2 # To prevent the gripper from touching the ground
                pos_in_arm=self.trans_listener.transformPoint("/locobot/base_link",aux)

                # p is the goal where the gripper should be in base_link frame
                p = PoseStamped()
                p.header.frame_id = "locobot/base_link"
                # x_g, y_g are the coordinates of the cube in 2-D
                p.pose.position.x = pos_in_arm.point.x # self.x_g
                p.pose.position.y = pos_in_arm.point.y # self.y_g
                p.pose.position.z - pos_in_arm.point.z # 0.02 # To prevent the gripper from touching the ground

                # quat = tf.transformations.quaternion_from_euler(0, self.theta_g, 0)
                p.pose.orientation.x = 0#quat[0]
                p.pose.orientation.y = 0#quat[1]
                p.pose.orientation.z = 0#quat[2]
                p.pose.orientation.w = 0#quat[3]
                rospy.loginfo("Moving to pick up cube")
                rospy.loginfo("x_g: {}, y_g: {}".format(self.x_g, self.y_g))
                rospy.loginfo("Goal: {}".format(p))
                self.move_arm_obj.move_gripper_down_to_grasp_callback(p)
                self.switch_mode(Mode.PARK)
      
            self.publish_control()
            rate.sleep()

    def shutdown_callback(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(cmd_vel)

if __name__ == "__main__":
    brain = Brain()
    rospy.on_shutdown(brain.shutdown_callback)
    brain.run()