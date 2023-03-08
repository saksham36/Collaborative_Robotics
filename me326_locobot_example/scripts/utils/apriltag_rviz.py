#!/usr/bin/env python3
'''
Written by: Saksham Consul 03/06/2023
Represent the location of Apriltag on RViz
'''
import rospy
import gazebo_msgs.msg
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class AprilTagRvizNode:
    def __init__(self):
        #Initialize node
        rospy.init_node('rviz_apriltag_node')
        rospy.wait_for_message("/gazebo/model_states", gazebo_msgs.msg.ModelStates)
        self.first_time = True
        self.model_list = []
        self.setup_sub()

    def setup_sub(self):
        self.gazebo_model_sub = rospy.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates, self.gazebo_model_callback, queue_size=10)
    
    def setup_pub(self):
        #Setup publisher for apriltag marker
        self.model_marker_pub = {}
        self.model_marker_pub.update({model : { "marker_pub" : rospy.Publisher("apriltag/"+model+"/marker", visualization_msgs.msg.Marker, queue_size=1),
                                             "msg" : visualization_msgs.msg.Marker(),
                                             }})
        # #Setup marker message
        # self.apriltag_marker_msg = Marker()
        # self.apriltag_marker_msg.header.frame_id = "locobot/odom"
        # self.apriltag_marker_msg.type = Marker.CUBE
        # self.apriltag_marker_msg.action = Marker.ADD
        # self.apriltag_marker_msg.scale.x = 0.1
        # self.apriltag_marker_msg.scale.y = 0.1
        # self.apriltag_marker_msg.scale.z = 0.1
        # self.apriltag_marker_msg.color.a = 1.0
        # self.apriltag_marker_msg.color.r = 1.0
        # self.apriltag_marker_msg.color.g = 0.0
        # self.apriltag_marker_msg.color.b = 0.0

        

    def apriltag_pose_callback(self, pose_msg):
        pass

    def gazebo_model_callback(self, model_state_msg):
        for i in range(len(model_state_msg.name)):
            if "Apriltag" in model_state_msg.name[i]:
                rospy.loginfo("Model state: %s", model_state_msg.name[i])
#=====================================
#               Main
#=====================================
if __name__ == "__main__":
    apriltag_rviz = AprilTagRvizNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("AprilTag RViz node shutting down")