#!/usr/bin/env python3
'''
Written by: Saksham Consul, Carlota Pares, Date: 2/21/2023
'''

import rospy
import numpy as np
import scipy as sp
from scipy import linalg
import geometry_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from apriltag_ros.msg import AprilTagDetectionArray


class OccupancyGridNode(object):

    def __init__(self):
        # Optitrack subscriber
        rospy.init_node('occupancy_grid_node')

        #Get parameters from ROS param server
        self.load_param()

        # # Flag to indicate if occupancy grid should be published
        # self.publish_occupancy_grid = False

        self.occupancy_grid = OccupancyGrid()
        
        # Initialize occupancy grid
        self.init_occupancy_grid()

        #Setup model subcriber and pose/odom publishers
        self.setup_sub_pub()
        
        self.ego_robot_pos = None
        self.other_robot_pos = None

        rospy.loginfo("Successfully initialized Occupancy Grid node!")
    
    #=====================================
    #         Initializes occupancy grid
    #=====================================
    def init_occupancy_grid(self):
        self.occupancy_grid.header.stamp = rospy.Time.now()
        self.occupancy_grid.header.frame_id = "map"
        self.occupancy_grid.info.resolution = self.resolution
        self.occupancy_grid.info.width = int(self.grid_size[0]/self.occupancy_grid.info.resolution)
        self.occupancy_grid.info.height = int(self.grid_size[1]/self.occupancy_grid.info.resolution)
        self.occupancy_grid.info.origin.position.x = self.origin[0]
        self.occupancy_grid.info.origin.position.y = self.origin[1]
        self.occupancy_grid.info.origin.position.z = 0
        self.occupancy_grid.info.origin.orientation.x = 0
        self.occupancy_grid.info.origin.orientation.y = 0
        self.occupancy_grid.info.origin.orientation.z = 0
        self.occupancy_grid.info.origin.orientation.w = 1
        self.occupancy_grid.data = [0] * self.occupancy_grid.info.width * self.occupancy_grid.info.height
    
    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))
    #=====================================
    #         Gets parameters from
    #         ROS parameter server
    #=====================================
    def load_param(self):
        self.ego_bot = rospy.get_param(rospy.search_param('ego_bot'), 'locobot_1')
        rospy.loginfo("Load param Ego robot: %s", self.ego_bot)
        self.other_bot = rospy.get_param(rospy.search_param('other_bot'), 'locobot_2')
        rospy.loginfo("Load param Other robot: %s", self.other_bot)
        self.grid_size = rospy.get_param('grid_size', [2,1.6])
        self.origin = rospy.get_param('origin', [0,0])
        self.resolution = rospy.get_param('resolution', 0.05)

    #=====================================
    #     Setup ROS subscribers and
    #  publishers for each tracked model
    #=====================================
    def setup_sub_pub(self):
        # Occupancy grid publisher
        self.occupancy_grid_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)

        # Cube position subscriber
        self.cube_sub = rospy.Subscriber('/locobot/camera_cube_locator', Marker, self.cube_callback)

        # Ego and other robot subscribers from Optitrack
        self.ego_robot_sub = rospy.Subscriber(f'/vrpn_client_node/{self.ego_bot}', PoseStamped, self.ego_robot_callback)
        self.other_robot_sub = rospy.Subscriber(f'/vrpn_client_node/{self.other_bot}', PoseStamped, self.other_robot_callback)
        
        
    #=====================================
    #         Callback for cube
    #=====================================
    def cube_callback(self, msg):
        # Get cube positions
        cube_points = msg.points

        for cube_point in cube_points:
            # Get cube position
            cube_pos = [cube_point.x, cube_point.y]

            # Get cube index
            cube_index = self.snap_to_grid(cube_pos)

            # Update occupancy grid
            self.occupancy_grid.data[cube_index] = 100
        
        # Publish occupancy grid
        self.occupancy_grid_pub.publish(self.occupancy_grid)

    #=====================================
    #         Callback for ego robot
    #=====================================
    def ego_robot_callback(self, msg):
        # Get ego robot position
        self.ego_robot_pos = [msg.pose.position.x, msg.pose.position.y]

        # Get ego robot index
        ego_robot_index = self.snap_to_grid(ego_robot_pos)

        # Update occupancy grid
        self.occupancy_grid.data[ego_robot_index] = 0

        # Publish occupancy grid
        self.occupancy_grid_pub.publish(self.occupancy_grid)

    
    #=====================================
    #         Callback for other robot
    #=====================================
    def other_robot_callback(self, msg):
        # Get other robot position
        self.other_robot_pos = [msg.pose.position.x, msg.pose.position.y]

        # Get other robot index
        other_robot_index = self.snap_to_grid(other_robot_pos)

        # Update occupancy grid
        self.occupancy_grid.data[other_robot_index] = 100

        # Publish occupancy grid
        self.occupancy_grid_pub.publish(self.occupancy_grid)

def set_grid_value(state, value):
    x, y = state
    grid_x = int((x - self.origin[0]) / self.resolution)
    grid_y = int((y - self.origin[1]) / self.resolution)
    if grid_y>0 and grid_x>0 and grid_x<self.width and grid_y<self.height:
                    p_total *= (1.0-max(0.0,float(self.probs[grid_y * self.width + grid_x])/100.0))
    x_index = int((x - origin[0])/resolution)
    y_index = int((y - origin[1])/resolution)
    return x_index, y_index        
#=====================================
#               Main
#=====================================
if __name__ == "__main__":
    grid = OccupancyGridNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Occupancy Grid node shutting down")