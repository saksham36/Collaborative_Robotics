#!/usr/bin/env python3

import rospy
import tf 
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D, Point
from visualization_msgs.msg import Marker
import yaml
import numpy as np
from occupancy_grid import CubeColor
import rospkg
import os

rp = rospkg.RosPack()
CONFIG_FILE = os.path.join(rp.get_path("me326_locobot_example"), "scripts", "config.yaml")
TEAM = "team_2"

class GoalFinder:
    def __init__(self):
        rospy.init_node("goal_finder", anonymous=True)

        self.tf = tf.TransformListener()

        # Perception Grid subscriber
        self.cube_sub = rospy.Subscriber('/perception_grid', OccupancyGrid, self.perception_callback)

        self.config = {}
        self.load_config()
        self.perception_grid = None

        # Goal publisher
        self.goal_pub = rospy.Publisher('/locobot/goal', Pose2D, queue_size=10)
        self.goal_marker_pub = rospy.Publisher('/locobot/marker_goal', Marker, queue_size=1)

    def load_config(self):
        with open(CONFIG_FILE, 'r') as f:
            config = yaml.safe_load(f)
        self.config['stations'] = [p for p in config['stations']]
        self.config['team_colors'] = [c for c in config[TEAM]]
        self.config['colors'] = [c for c in config['colors']]
        self.config['resources'] = np.array(config['resources'])

    def perception_callback(self, msg):
        self.perception_grid = msg
        grid = self.get_grid_as_np(self.perception_grid)
        robot_pos = self.get_robot_pos(grid)
        self.mask_grid(grid)

        ## TODO ##
        # Implement station - cubes logic
        # Now returning closest cube to robot

        # Find closest cube to robot
        cubes = np.where((grid != 0) & (grid != 100))
        cubes = np.hstack([cubes[0], cubes[1]]).reshape(-1,2)
        try:
            closest_cube = np.argmin(np.linalg.norm(cubes - robot_pos, axis=1))
        except:
            return
        cube_pos = cubes[closest_cube]
        x,y = self.get_xy_from_cell_index(cube_pos)
        self.publish_goal(x,y)

    def publish_goal(self, x, y):
        goal = Pose2D()
        goal.x = x
        goal.y = y
        (_,rotation) = self.tf.lookupTransform('locobot/odom', \
            'locobot/base_footprint', rospy.Time(0))
        euler = euler_from_quaternion(rotation)
        goal.theta = euler[2]
        self.goal_pub.publish(goal)

        goal_marker = Marker()
        goal_marker.header.frame_id = "locobot/odom"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.type = Marker.ARROW
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = x
        goal_marker.pose.position.y = y
        goal_marker.pose.position.z = 0.1
        goal_marker.pose.orientation.x = rotation[0]
        goal_marker.pose.orientation.y = rotation[1]
        goal_marker.pose.orientation.z = rotation[2]
        goal_marker.pose.orientation.w = rotation[3]
        goal_marker.scale.x = 0.3
        goal_marker.scale.y = 0.1
        goal_marker.scale.z = 0.1
        goal_marker.color.a = 1.0
        goal_marker.color.r = 0.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        self.goal_marker_pub.publish(goal_marker)

    
    def get_xy_from_cell_index(self, index):
        x = index[1]*self.perception_grid.info.resolution + self.perception_grid.info.origin.position.x
        y = index[0]*self.perception_grid.info.resolution + self.perception_grid.info.origin.position.y
        return x, y

    def get_grid_as_np(self, grid):
        return np.array(grid.data).reshape(grid.info.height, grid.info.width)

    def get_robot_pos(self, grid):
        x, y = np.where(grid == 100)
        return np.array([x[0],y[0]])
    
    def mask_grid(self, grid):
        for color in self.config['colors']:
            if color not in self.config['team_colors']:
                if color == "red":
                    val = CubeColor.RED
                elif color == "green":
                    val = CubeColor.GREEN
                elif color == "blue":
                    val = CubeColor.BLUE
                elif color == "yellow":
                    val = CubeColor.YELLOW
                xs, ys = np.where(grid == val)
                for x, y in zip(xs,ys):
                    grid[x,y] = 0


if __name__ == "__main__":
    gf = GoalFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Goal finder node shutting down")
