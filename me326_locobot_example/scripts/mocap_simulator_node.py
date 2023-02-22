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
import nav_msgs.msg
import copy

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

        self.fixed_frame_id = rospy.get_param('~fixed_frame_id', "optitrack")
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
        
        for model in self.model_list:
            rospy.loginfo("Initializing subject "+model)
            self.model_pub.update({model : { "pose_stamped_pub" : rospy.Publisher("vrpn_client_node/"+model+"/pose", geometry_msgs.msg.PoseStamped, queue_size=100),
                                             "pose_stamped_msg" : geometry_msgs.msg.PoseStamped(),
                                             }})
            #init message headers
            self.model_pub[model]["pose_stamped_msg"].header.frame_id = self.fixed_frame_id

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
                    if model_state_msg.name[i] == model:
                        #Get model pose from Gazebo message
                        self.model_pub[model]["pose_stamped_msg"].pose = model_state_msg.pose[i]
                        #Timestamp messages
                        self.model_pub[model]["pose_stamped_msg"].header.stamp = curr_time
                        #Publish messages
                        self.model_pub[model]["pose_stamped_pub"].publish(self.model_pub[model]["pose_stamped_msg"])
                        
#=====================================
#               Main
#=====================================
if __name__ == "__main__":
    mocapSimulator = MocapSimulatorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Optitrack interface node shutting down")