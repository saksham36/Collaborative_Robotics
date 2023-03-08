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

rp = rospkg.RosPack()
CONFIG_FILE = os.path.join(rp.get_path("me326_locobot_example"), "scripts", "config.yaml")
TEAM = "team_2"

class GoalFinder:
    def __init__(self):
        rospy.init_node("goal_finder", anonymous=True)

        self.tf = tf.TransformListener()

        # TODO: THis is only for debugging. Set's the goal
        self.x_g = None
        self.y_g = None

        self.config = {}
        self.stations_marker_pub = {}
        self.load_config()
        self.perception_grid = None

        self.station_pub_flag = False

        # Goal publisher
        self.goal_pub = rospy.Publisher('/locobot/goal', Pose2D, queue_size=10)
        self.goal_marker_pub = rospy.Publisher('/locobot/marker_goal', Marker, queue_size=1)
        

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
            self.station_pub_flag = True
            self.publish_goal(station[0], station[1], False)

    def perception_callback(self, msg):
        # rospy.loginfo("Received perception grid")
        self.perception_grid = msg
        grid = self.get_grid_as_np(self.perception_grid)
        try:
            robot_pos = self.get_robot_pos(grid)
        except:
            rospy.loginfo("Robot not in grid")
            return
        self.mask_grid(grid)

        ## TODO ##
        # Implement station - cubes logic
        # Now returning closest cube to robot

        # Find closest cube to robot
        cubes = np.where((grid != 0) & (grid != 100))
        cubes = np.hstack([cubes[0], cubes[1]]).reshape(-1,2)
        try:
            closest_cube = np.argmin(np.linalg.norm(cubes - robot_pos, axis=1))
        except Exception as e:
            rospy.loginfo("No cubes in grid")
            rospy.loginfo("Error: " + str(e))
            return
        cube_pos = cubes[closest_cube]
        if self.x_g is None or self.y_g is None: # TODO: This is only for picking 1 cube
            x,y = self.get_xy_from_cell_index(cube_pos)
            self.x_g = x
            self.y_g = y
        self.publish_goal(self.x_g,self.y_g, True)
        self.publish_stations()
        
    def publish_stations(self):
        for i in range(len(self.config['stations'])):
            self.stations_marker_pub[i]['marker_pub'].publish(self.stations_marker_pub[i]['msg'])

    def publish_goal(self, x, y, from_perception=True):
        goal = Pose2D()
        if self.station_pub_flag:# and from_perception == False:
            rospy.loginfo("Publishing station as goal")
            goal.x = -0.5# self.x_g
            goal.y = -0.5# self.y_g
        else:
            rospy.loginfo("Publishing station as cube")
            # goal.x = x
            # goal.y = y
            goal.x = 3
            goal.y = 3
        (_,rotation) = self.tf.lookupTransform('locobot/odom', \
            'locobot/base_link', rospy.Time(0))
        euler = euler_from_quaternion(rotation)
        goal.theta = euler[2]
        # rospy.loginfo("Publishing goal: " + str(goal) + " to /locobot/goal")
        self.goal_pub.publish(goal)

        goal_marker = Marker()
        goal_marker.header.frame_id = "locobot/odom"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.type = Marker.ARROW
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = 1 # x
        goal_marker.pose.position.y = 1 # y
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
