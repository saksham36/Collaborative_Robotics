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

############# Occupancy Grid Example #############

#   * * * * *
#   * * * * *
#   * * O * *
#   * * * * *
#   * * * * *

# origin is set at the middle of the grid (we must set the grid size to be odd)

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
        
        # Initialize occupancy grid
        self.init_occupancy_grid()

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
        self.occupancy_grid_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)

        # Cube position subscriber
        self.cube_sub = rospy.Subscriber('/locobot/camera_cube_locator', Marker, self.cube_callback)

        # Ego and other robot subscribers from Optitrack
        self.ego_robot_sub = rospy.Subscriber(f'/vrpn_client_node/{self.ego_bot}', PoseStamped, self.ego_robot_callback)
        self.other_robot_sub = rospy.Subscriber(f'/vrpn_client_node/{self.other_bot}', PoseStamped, self.other_robot_callback)
        
    #=====================================
    #         Initializes occupancy grid
    #=====================================
    def init_occupancy_grid(self):
        self.occupancy_grid.header.stamp = rospy.Time.now()
        self.occupancy_grid.header.frame_id = "locobot/base_link"
        self.occupancy_grid.info.resolution = self.resolution
        self.occupancy_grid.info.width = int(self.grid_size[0] / self.resolution) # num cells in x
        self.occupancy_grid.info.height = int(self.grid_size[1] / self.resolution) # num cells in y
        print('cells x', self.occupancy_grid.info.width)
        print('cells y', self.occupancy_grid.info.height)
        # origin --> real-world pose of the cell (0,0) in the map
        self.occupancy_grid.info.origin.position.x = self.origin[0] - self.grid_size[0]/2
        self.occupancy_grid.info.origin.position.y = self.origin[1] - self.grid_size[1]/2
        print('origin', self.occupancy_grid.info.origin.position.x, self.occupancy_grid.info.origin.position.y)
        self.occupancy_grid.info.origin.position.z = 0
        self.occupancy_grid.info.origin.orientation.x = 0
        self.occupancy_grid.info.origin.orientation.y = 0
        self.occupancy_grid.info.origin.orientation.z = 0
        self.occupancy_grid.info.origin.orientation.w = 1
        self.occupancy_grid.data = [0] * self.occupancy_grid.info.width * self.occupancy_grid.info.height
    
    #=====================================
    #         Callback for cube
    #=====================================
    def cube_callback(self, msg):
        # Get cube positions
        cube_points = msg.points

        for cube_point in cube_points:
            # Get cube position
            # print(cube_point.x, cube_point.y)
            cube_pos = self.transform_point(msg.header.frame_id, self.occupancy_grid.header.frame_id, cube_point, msg.header.stamp)
            # print(cube_pos.point.x, cube_pos.point.y)

            # Update occupancy grid
            self.update_occupancy_grid(cube_pos.point.x, cube_pos.point.y, 100)
        
        # Publish occupancy grid
        self.occupancy_grid_pub.publish(self.occupancy_grid)

    #=====================================
    #         Callback for ego robot
    #=====================================
    def ego_robot_callback(self, msg):
        # Get ego robot position
        self.ego_robot_pos = [msg.pose.position.x, msg.pose.position.y]

        # Update occupancy grid
        self.update_occupancy_grid(self.ego_robot_pos[0], self.ego_robot_pos[1], 0)

        # Publish occupancy grid
        self.occupancy_grid_pub.publish(self.occupancy_grid)

    
    #=====================================
    #         Callback for other robot
    #=====================================
    def other_robot_callback(self, msg):
        # Get other robot position
        self.other_robot_pos = [msg.pose.position.x, msg.pose.position.y]

        # Update occupancy grid
        self.update_occupancy_grid(self.other_robot_pos[0], self.other_robot_pos[1], 100)
        print("other pos", self.other_robot_pos)

        # Publish occupancy grid
        self.occupancy_grid_pub.publish(self.occupancy_grid)
    
    def get_occupancy_grid_as_np(self):
        return np.array(self.occupancy_grid.data).reshape(self.occupancy_grid.info.height, self.occupancy_grid.info.width)
    
    def get_xy_from_cell_index(self, index):
        x = index[0]*self.resolution + self.occupancy_grid.info.origin.position.x
        y = index[1]*self.resolution + self.occupancy_grid.info.origin.position.y
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
    
    def update_occupancy_grid(self, x, y, value):
        x_index, y_index = self.get_cell_index_from_xy(x, y)
        if self.is_in_gridmap([x_index, y_index]):
            self.occupancy_grid.data[y_index*self.occupancy_grid.info.width + x_index] = value
            
    def transform_point(self, org_frame, dest_frame, point, ts):
    	self.tf.waitForTransform(org_frame, dest_frame, ts, rospy.Duration(4.0))
    	aux = PointStamped()
    	aux.header.frame_id = org_frame
    	aux.header.stamp = ts
    	aux.point.x = point.x
    	aux.point.y = point.y
    	aux.point.z = 0.0
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

