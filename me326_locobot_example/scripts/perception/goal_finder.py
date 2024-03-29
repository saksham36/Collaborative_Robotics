#!/usr/bin/env python3

import rospy
import tf 
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D, Point
import visualization_msgs.msg
from visualization_msgs.msg import Marker
import yaml
import numpy as np
from occupancy_grid import CubeColor
import rospkg
import os
import matplotlib.pyplot as plt

rp = rospkg.RosPack()
CONFIG_FILE = os.path.join(rp.get_path("me326_locobot_example"), "scripts", "config.yaml")
TEAM = "team_1"

class GoalFinder:
    def __init__(self):
        rospy.init_node("goal_finder", anonymous=True)

        self.tf = tf.TransformListener()

        self.x_g = None
        self.y_g = None
        self.dropped_cubes = []

        self.config = {}
        self.stations_marker_pub = {}
        self.load_config()
        self.perception_grid = None

        self.station_pub_flag = None

        # Goal publisher
        self.goal_pub = rospy.Publisher('/locobot/goal', Pose2D, queue_size=10)
        self.goal_marker_pub = rospy.Publisher('/locobot/marker_goal', Marker, queue_size=1)

        self.cubes = [(0.922, -0.7169), (-0.3574, -1.1008), (-1.0035, -0.0437), (-1.482, -1.0328), (-1.824, 0.7602) ]
        self.num_cubes = 0
        

        # Perception Grid subscriber
        self.cube_sub = rospy.Subscriber('/perception_grid', OccupancyGrid, self.perception_callback)
        self.ask_for_station_pub = rospy.Subscriber('/locobot/ask_for_station', Bool, self.ask_for_station_callback)       
        
    def load_config(self):
        with open(CONFIG_FILE, 'r') as f:
            config = yaml.safe_load(f)
        self.config['stations'] = [p for p in config['stations']]
        self.config['team_colors'] = [c for c in config[TEAM]]
        self.config['colors'] = [c for c in config['colors']]
        self.config['resources'] = np.array(config['resources'])

        self.setup_station_pub()
    
    def setup_station_pub(self):
        for i, station in enumerate(self.config['stations']):
            self.stations_marker_pub.update({i : { "marker_pub" : rospy.Publisher("station/"+str(i)+"/marker", visualization_msgs.msg.Marker, queue_size=1),
                                                   "msg" : visualization_msgs.msg.Marker(),
                                                   }})
            self.stations_marker_pub[i]['msg'].header.frame_id = "locobot/odom"
            self.stations_marker_pub[i]['msg'].type = Marker.SPHERE
            self.stations_marker_pub[i]['msg'].action = Marker.ADD
            self.stations_marker_pub[i]['msg'].scale.x = 0.1
            self.stations_marker_pub[i]['msg'].scale.y = 0.1
            self.stations_marker_pub[i]['msg'].scale.z = 0.01
            self.stations_marker_pub[i]['msg'].color.a = 1.0
            # Color: (1, 0, 0), (0, 1, 0), (0, 0, 1)
            self.stations_marker_pub[i]['msg'].color.r = 1 if i%3==0 else 0 
            self.stations_marker_pub[i]['msg'].color.g = 1 if (i+2)%3==0 else 0
            self.stations_marker_pub[i]['msg'].color.b = 1 if (i+1)%3==0 else 0 
            self.stations_marker_pub[i]['msg'].pose.position.x = station[0]
            self.stations_marker_pub[i]['msg'].pose.position.y = station[1]
            self.stations_marker_pub[i]['msg'].pose.position.z = 0.0

    def ask_for_station_callback(self, msg):
        station = self.config['stations'][0] #TODO: Station selection algo
        self.station_pub_flag = msg.data
        if self.station_pub_flag:
            self.dropped_cubes.append((self.x_g,self.y_g))
            self.x_g = station[0]
            self.y_g = station[1]
            self.publish_goal(self.x_g,self.y_g)
            self.num_cubes += 1
        rospy.loginfo("station_pub_flag: {}".format(self.station_pub_flag))

    def perception_callback(self, msg):
        # rospy.loginfo("Received perception grid")
        self.perception_grid = msg
        grid = self.get_grid_as_np(self.perception_grid)
        try:
            robot_pos = self.get_robot_pos(grid)
        except:
            rospy.loginfo("Robot not in grid")
            return

        grid = self.mask_grid(grid)

        ## TODO ##
        # Implement station - cubes logic
        # Now returning closest cube to robot

        # Find closest cube to robot
        cubes = np.where((grid != 0) & (grid != 100))
        cubes = np.hstack([cubes[0], cubes[1]]).reshape(-1,2)

        for cube in cubes:
            if tuple(cube.tolist()) in self.dropped_cubes:
                aux = np.where((cubes == cube).all(axis=1))
                if len(aux[0]) > 1:
                    cubes = np.delete(cubes, aux[0][1:], axis=0)
        try:
            closest_cube = np.argmin(np.linalg.norm(cubes - robot_pos, axis=1))
        except Exception as e:
            return
        cube_pos = cubes[closest_cube]
        if self.x_g is None or self.y_g is None or self.station_pub_flag is not None: # This is only for picking 1 cube
            x,y = self.get_xy_from_cell_index(cube_pos)
            self.x_g = x
            self.y_g = y
        self.publish_goal(self.x_g,self.y_g)
        self.publish_stations()
        
    def publish_stations(self):
        for i in range(len(self.config['stations'])):
            self.stations_marker_pub[i]['marker_pub'].publish(self.stations_marker_pub[i]['msg'])

    def publish_goal(self, x, y):
        if self.station_pub_flag is None or self.x_g is None or self.y_g is None:
            return
        goal = Pose2D()
        if self.station_pub_flag:
            goal.x = -1 #x
            goal.y = 0 #y
            self.x_g, self.y_g = None, None
            # goal.x = -0.5
            # goal.y = -0.5
        else:
            # x = 1
            # y = 1
            goal.x = self.cubes[self.num_cubes][0]
            goal.y = self.cubes[self.num_cubes][1]
            # goal.x = 1
            # goal.y = 1
        (_,rotation) = self.tf.lookupTransform('locobot/odom', \
            'locobot/base_link', rospy.Time(0))
        euler = euler_from_quaternion(rotation)
        goal.theta = euler[2]
        self.goal_pub.publish(goal)

        goal_marker = Marker()
        goal_marker.header.frame_id = "locobot/odom"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.type = Marker.ARROW
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = goal.x
        goal_marker.pose.position.y = goal.y
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
        grid = np.array(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
        return grid

    def get_robot_pos(self, grid):
        x, y = np.where(grid == 100)
        return np.array([x[0],y[0]])
    
    def mask_grid(self, grid):
        for color in self.config['colors']:
            if color not in self.config['team_colors']:
                if color == "red":
                    val = CubeColor.RED.value
                elif color == "green":
                    val = CubeColor.GREEN.value
                elif color == "blue":
                    val = CubeColor.BLUE.value
                elif color == "yellow":
                    val = CubeColor.YELLOW.value
                xs, ys = np.where(grid == val)
                for x, y in zip(xs,ys):
                    grid[x,y] = 0
        return grid


if __name__ == "__main__":
    gf = GoalFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Goal finder node shutting down")
