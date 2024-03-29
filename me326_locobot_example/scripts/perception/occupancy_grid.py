#!/usr/bin/env python3
'''
Written by: Saksham Consul, Carlota Pares, Date: 2/21/2023
'''

import rospy
import numpy as np
import scipy as sp
from scipy import linalg
import geometry_msgs
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from enum import Enum

class CubeColor(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3
    YELLOW = 4

############# Occupancy Grid Example #############

#   * * * * *
#   * * * * *
#   * * O * *
#   * * * * *
#   * * * * *

# origin is set at the middle of the grid

##################################################


class OccupancyGridNode(object):

    def __init__(self):
        # Optitrack subscriber
        rospy.init_node('occupancy_grid_node')

        #Get parameters from ROS param server
        self.load_param()

        # # Flag to indicate if occupancy grid should be published
        # self.publish_occupancy_grid = False

        self.occupancy_grid = OccupancyGrid()
        self.perception_grid = OccupancyGrid()
        self.cube_thresh = 0.1
        
        # Initialize occupancy grid
        self.init_grids()

        #Setup model subcriber and pose/odom publishers
        self.setup_sub_pub()
        
        self.ego_robot_pos = None
        self.other_robot_pos = None
        
        self.tf = tf.TransformListener()

        rospy.loginfo("Successfully initialized Occupancy Grid node!")

        
    
    #=====================================
    #         Gets parameters from
    #         ROS parameter server
    #=====================================
    def load_param(self):
        self.ego_bot = rospy.get_param(rospy.search_param('ego_bot'), 'locobot_1')
        rospy.loginfo("Load param Ego robot: %s", self.ego_bot)
        self.other_bot = rospy.get_param(rospy.search_param('other_bot'), 'locobot_2')
        rospy.loginfo("Load param Other robot: %s", self.other_bot)
        self.grid_size = rospy.get_param('grid_size', [4,3.2]) # 2m x 1.6m (real size)
        self.origin = rospy.get_param('origin', [0,0])
        self.resolution = rospy.get_param('resolution', 0.05) # 5 cm resolution

    #=====================================
    #     Setup ROS subscribers and
    #  publishers for each tracked model
    #=====================================
    def setup_sub_pub(self):
        # Occupancy grid publisher
        self.occupancy_grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        # perception grid publisher
        self.perception_grid_pub = rospy.Publisher('/perception_grid', OccupancyGrid, queue_size=10)

        # Cube position subscriber
        self.cube_sub = rospy.Subscriber('/locobot/camera_cube_locator', Marker, self.cube_callback)

        # Ego and other robot subscribers from Optitrack
        self.ego_robot_sub = rospy.Subscriber(f'/vrpn_client_node/{self.ego_bot}/pose', PoseStamped, self.ego_robot_callback)
        self.other_robot_sub = rospy.Subscriber(f'/vrpn_client_node/{self.other_bot}/pose', PoseStamped, self.other_robot_callback)
        
    #=====================================
    #         Initializes occupancy grid
    #=====================================
    def init_grids(self):
        self.occupancy_grid.header.stamp = rospy.Time.now()
        self.perception_grid.header.stamp = rospy.Time.now()
        self.occupancy_grid.header.frame_id = "locobot/odom"
        self.perception_grid.header.frame_id = "locobot/odom"
        self.occupancy_grid.info.resolution = self.resolution
        self.perception_grid.info.resolution = self.resolution
        self.occupancy_grid.info.width = int(self.grid_size[0] / self.resolution) # num cells in x
        self.perception_grid.info.width = int(self.grid_size[0] / self.resolution)
        self.occupancy_grid.info.height = int(self.grid_size[1] / self.resolution) # num cells in y
        self.perception_grid.info.height = int(self.grid_size[1] / self.resolution)
        # origin --> real-world pose of the cell (0,0) in the map
        self.occupancy_grid.info.origin.position.x = self.origin[0] - self.grid_size[0]/2
        self.occupancy_grid.info.origin.position.y = self.origin[1] - self.grid_size[1]/2
        self.perception_grid.info.origin.position.x = self.origin[0] - self.grid_size[0]/2
        self.perception_grid.info.origin.position.y = self.origin[1] - self.grid_size[1]/2
        self.occupancy_grid.info.origin.position.z = 0
        self.perception_grid.info.origin.position.z = 0
        self.occupancy_grid.info.origin.orientation.x = 0
        self.occupancy_grid.info.origin.orientation.y = 0
        self.occupancy_grid.info.origin.orientation.z = 0
        self.occupancy_grid.info.origin.orientation.w = 1
        self.perception_grid.info.origin.orientation.x = 0
        self.perception_grid.info.origin.orientation.y = 0
        self.perception_grid.info.origin.orientation.z = 0
        self.perception_grid.info.origin.orientation.w = 1
        self.occupancy_grid.data = [0] * self.occupancy_grid.info.width * self.occupancy_grid.info.height
        self.perception_grid.data = [0] * self.perception_grid.info.width * self.perception_grid.info.height
    
    #=====================================
    #         Callback for cube
    #=====================================
    def cube_callback(self, msg):
        # Get cube positions
        cube_points = msg.points
        cube_colors = msg.colors

        for ii, cube_point in enumerate(cube_points):
            # Get cube position
            cube_pos = self.transform_point(msg.header.frame_id, self.occupancy_grid.header.frame_id, cube_point, msg.header.stamp)

            color = (cube_colors[ii].r, cube_colors[ii].g, cube_colors[ii].b)
            if color == (1,0,0):
                value = CubeColor.RED.value
            elif color == (0,1,0):
                value = CubeColor.GREEN.value
            elif color == (0,0,1):
                value = CubeColor.BLUE.value
            elif color == (1,1,0):
                value = CubeColor.YELLOW.value
            else:
                value = -1

            # Update occupancy grid
            self.update_grid(self.occupancy_grid,cube_pos.point.x, cube_pos.point.y, 100)
            self.update_grid(self.perception_grid,cube_pos.point.x, cube_pos.point.y, value)
        
        # Publish occupancy grid
        self.occupancy_grid_pub.publish(self.occupancy_grid)
        # Publish perception grid
        self.perception_grid_pub.publish(self.perception_grid)

    #=====================================
    #         Callback for ego robot
    #=====================================
    def ego_robot_callback(self, msg):
        # Get ego robot position
        self.ego_robot_pos = [msg.pose.position.x, msg.pose.position.y]

        # Update occupancy grid
        self.update_grid(self.occupancy_grid, self.ego_robot_pos[0], self.ego_robot_pos[1], 0)
        self.update_grid(self.perception_grid, self.ego_robot_pos[0], self.ego_robot_pos[1], 100)

        # Publish occupancy grid
        self.occupancy_grid_pub.publish(self.occupancy_grid)
        self.perception_grid_pub.publish(self.perception_grid)

    
    #=====================================
    #         Callback for other robot
    #=====================================
    def other_robot_callback(self, msg):
        # Get other robot position
        self.other_robot_pos = [msg.pose.position.x, msg.pose.position.y]

        # Update occupancy grid
        self.update_grid(self.occupancy_grid, self.other_robot_pos[0], self.other_robot_pos[1], 100)

        # Publish occupancy grid
        self.occupancy_grid_pub.publish(self.occupancy_grid)
    
    def get_xy_from_cell_index(self, index):
        x = index[1]*self.resolution + self.occupancy_grid.info.origin.position.x
        y = index[0]*self.resolution + self.occupancy_grid.info.origin.position.y
        return x, y
    
    def get_cell_index_from_xy(self, x, y):
        x_index = int((x - self.occupancy_grid.info.origin.position.x)/self.resolution)
        y_index = int((y - self.occupancy_grid.info.origin.position.y)/self.resolution)
        return x_index, y_index
    
    def is_in_gridmap(self, index):
        if index[0] >= 0 and index[0] < self.occupancy_grid.info.width and index[1] >= 0 and index[1] < self.occupancy_grid.info.height:
            return True
        return False
    
    def get_closest_cell(self, x, y):
        x_index, y_index = self.get_cell_index_from_xy(x, y)
        if self.is_in_gridmap([x_index, y_index]):
            return x_index, y_index
        return None
    
    def is_cell_free(self, grid, x, y):
        num_cells = int(self.cube_thresh/grid.info.resolution)
        for ii in range(-num_cells, num_cells+1):
            for jj in range(-num_cells, num_cells+1):
                if self.is_in_gridmap([x+ii, y+jj]):
                    if grid.data[(y+jj)*grid.info.width + (x+ii)] != 0:
                        return False
        return True
    
    def update_grid(self, grid, x, y, value):
        x_index, y_index = self.get_cell_index_from_xy(x, y)
        if self.is_cell_free(grid, x_index, y_index):
            if self.is_in_gridmap([x_index, y_index]):
                grid.data[y_index*grid.info.width + x_index] = value
            
    def transform_point(self, org_frame, dest_frame, point, ts):
        self.tf.waitForTransform(org_frame, dest_frame, ts, rospy.Duration(4.0))
        aux = PointStamped()
        aux.header.frame_id = org_frame
        aux.header.stamp = ts
        aux.point.x = point.x
        aux.point.y = point.y
        aux.point.z = point.z
        return self.tf.transformPoint(dest_frame, aux)

#=====================================
#               Main
#=====================================
if __name__ == "__main__":
    grid = OccupancyGridNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Occupancy Grid node shutting down")

