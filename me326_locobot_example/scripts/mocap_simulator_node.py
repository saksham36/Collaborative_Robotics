#!/usr/bin/env python3
'''
Written by: Saksham Consul 2/10/2023
Simulate a mocap system by providing the same interface as the Optitrack motion_capture_system
Provides the same ROS topics as Optitrack.

Modified the implementation from https://github.com/KTH-SML/motion_capture_simulator
'''
import sys
import decimal
import rospy
import gazebo_msgs.msg
import gazebo_msgs.srv
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import Point, PointStamped
import visualization_msgs.msg
from visualization_msgs.msg import Marker
import nav_msgs.msg
import copy
import tf
import numpy as np

from tf.transformations import *

class MocapSimulatorNode:
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #Initialize node
        rospy.init_node('mocap_simulator_node')

        rospy.loginfo("Mocap simulator node: waiting for Gazebo feedback and for model to spawn...")
        rospy.wait_for_message("/gazebo/model_states", gazebo_msgs.msg.ModelStates)
        rospy.sleep(2)
        self.prev_time = rospy.Time.now()

        #Get parameters from ROS param server
        self.load_param()

        #Setup model subcriber and pose/odom publishers
        self.setup_sub_pub()

        rospy.loginfo("Successfully initialized Optitrack interface node!")

    #=====================================
    #         Gets parameters from
    #         ROS parameter server
    #=====================================
    def load_param(self):
        #Get and set frame rate
        self.frame_rate = rospy.get_param('~frame_rate', 0)
        if (self.frame_rate <= 0) or (self.frame_rate > 100):
            self.frame_rate = 100
        self.frame_delay = 1/float(self.frame_rate)

        # self.fixed_frame_id = rospy.get_param('~fixed_frame_id', "optitrack")
        self.fixed_frame_id = "locobot/odom"
        self.model_list = rospy.get_param('~model_list', [])

    #=====================================
    #     Setup ROS subscribers and
    #  publishers for each tracked model
    #=====================================
    def setup_sub_pub(self):
        #----------------------------------
        # Create publishers for each model
        #----------------------------------
        #Publishers are stored in a dictionnary with the model name as key
        self.model_pub = {}
        self.model_marker_pub = {}
        
        for model in self.model_list:
            rospy.loginfo("Initializing subject "+model)
            self.model_pub.update({model : { "pose_stamped_pub" : rospy.Publisher("vrpn_client_node/"+model+"/pose", geometry_msgs.msg.PoseStamped, queue_size=1),
                                             "pose_stamped_msg" : geometry_msgs.msg.PoseStamped(),
                                             }})
            self.model_marker_pub.update({model : { "marker_pub" : rospy.Publisher("vrpn_client_node/"+model+"/marker", visualization_msgs.msg.Marker, queue_size=1),
                                             "msg" : visualization_msgs.msg.Marker(),
                                             }})
            #init message headers
            self.model_pub[model]["pose_stamped_msg"].header.frame_id = self.fixed_frame_id
            self.model_marker_pub[model]["msg"].header.frame_id = self.fixed_frame_id

        #------------------------------------------
        # Initialize Gazebo model state subscriber
        #------------------------------------------
        self.gazebo_model_sub = rospy.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates, self.gazebo_model_callback, queue_size=10)

    #=====================================
    #          Callback function 
    #      for Gazebo model states
    #=====================================
    def gazebo_model_callback(self, model_state_msg):
        curr_time = rospy.Time.now()
        #Only process callback at simulated motion capture system frequency
        if curr_time.to_sec() > (self.prev_time.to_sec() + self.frame_delay):
            self.prev_time = curr_time
            for i in range(len(model_state_msg.name)):
                for model in self.model_list:
                    # rospy.loginfo("Model state: %s", model_state_msg.name[i])
                    # rospy.loginfo("Model "+model+" found in Gazebo model state message!")
                    if model_state_msg.name[i] == model or model_state_msg.name[i] == "red_cube_6":
                        #Get model pose from Gazebo message
                        self.model_pub[model]["pose_stamped_msg"].pose = model_state_msg.pose[i]
                        start = model_state_msg.pose[i].position
                        self.model_marker_pub[model]["msg"].pose.position.x = start.x
                        self.model_marker_pub[model]["msg"].pose.position.y = start.y
                        self.model_marker_pub[model]["msg"].pose.position.z = 0
                        self.model_marker_pub[model]["msg"].color.r = 0.0
                        self.model_marker_pub[model]["msg"].color.g = 0.0
                        self.model_marker_pub[model]["msg"].color.b = 0.0
                        self.model_marker_pub[model]["msg"].color.a = 1.0
                        self.model_marker_pub[model]["msg"].scale.x = 0.7
                        self.model_marker_pub[model]["msg"].scale.y = 0.07
                        self.model_marker_pub[model]["msg"].scale.z = 0.07
                        self.model_marker_pub[model]["msg"].ns = model
                        quaternion = tf.transformations.quaternion_from_euler(0, -np.pi/2, start.z)
                        self.model_marker_pub[model]["msg"].pose.orientation.x = quaternion[0]
                        self.model_marker_pub[model]["msg"].pose.orientation.y = quaternion[1]
                        self.model_marker_pub[model]["msg"].pose.orientation.z = quaternion[2]
                        self.model_marker_pub[model]["msg"].pose.orientation.w = quaternion[3]
                        
                        self.model_marker_pub[model]["msg"].type = Marker.ARROW
                        #Timestamp messages
                        self.model_pub[model]["pose_stamped_msg"].header.stamp = curr_time
                        self.model_marker_pub[model]["msg"].header.stamp = curr_time
                        #Publish messages
                        self.model_pub[model]["pose_stamped_pub"].publish(self.model_pub[model]["pose_stamped_msg"])
                        self.model_marker_pub[model]["marker_pub"].publish(self.model_marker_pub[model]["msg"])
                        
#=====================================
#               Main
#=====================================
if __name__ == "__main__":
    mocapSimulator = MocapSimulatorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Optitrack interface node shutting down")
