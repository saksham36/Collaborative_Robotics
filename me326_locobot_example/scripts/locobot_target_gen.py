#!/usr/bin/env python3
'''
Written by: Saksham Consul 2/1/2023
'''
import rospy
import numpy as np
import scipy as sp
from scipy import linalg
import geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class CircleTarget(object):
    """Class for generating target to be followed 

    This script demonstrates how to track a circle using proportional Control
    The pose of the Locobot is defined by its (x,y) position 
    """
    def __init__(self):
        self.t_init = rospy.Time.now()
        self.current_target = self.target_pose_calculator(rospy.Time.now()-self.t_init)

        self.target_pose_publisher = rospy.Publisher("/locobot/mobile_base/target_pose", Pose, queue_size=10) # topic to publish to update target goal
        self.target_pose_visual = rospy.Publisher("/locobot/mobile_base/target_pose_visual",Marker, queue_size=1)
        self.goal_reached_error = 0.005

    def target_pose_calculator(self, d):
        t = d.to_sec()
        rospy.loginfo("Time: %s", t)
        target = Pose()
        if t <= 20:
            theta = 2*np.pi*t/10
            target.position.x = 0.5 * np.cos(theta)
            target.position.y = 0.5 * np.sin(theta)
            target.position.z = 0
            target.orientation.x = 0
            target.orientation.y = 0 
            target.orientation.z = np.sin(((np.pi/2)+theta)/2)
            target.orientation.w = np.cos(((np.pi/2)+theta)/2)
        else:
            target.position.x = 0.5 
            target.position.y = 0.0
            target.position.z = 0
            target.orientation.x = 0
            target.orientation.y = 0
            target.orientation.z = np.sin(np.pi/4)
            target.orientation.w = np.sin(np.pi/4)
        return target


    def pub_target_point_marker(self):
        #this is putting the marker in the world frame (http://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29)
        marker = Marker()
        marker.header.frame_id = "locobot/odom" #this will be the world frame for the real robot
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.ARROW
        # Set the marker scale
        marker.scale.x = 0.3  # arrow length
        marker.scale.y = 0.1 #arrow width
        marker.scale.z = 0.1 #arrow height

        # Set the marker pose
        marker.pose.position.x = self.current_target.position.x  # center of the sphere in base_link frame
        marker.pose.position.y = self.current_target.position.y
        marker.pose.position.z = self.current_target.position.z
        marker.pose.orientation.x = self.current_target.orientation.x
        marker.pose.orientation.y = self.current_target.orientation.y
        marker.pose.orientation.z = self.current_target.orientation.z
        marker.pose.orientation.w = self.current_target.orientation.w

        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 0.0 #red
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Publish the marker
        self.target_pose_visual.publish(marker)

     
    def update_target(self):
        while not rospy.is_shutdown():
            self.current_target = self.target_pose_calculator(rospy.Time.now()-self.t_init)
            self.target_pose_publisher.publish(self.current_target)
            #publish the point P and target pose markers for visualization in Rviz:
            self.pub_target_point_marker()


def main():
    rospy.init_node('locobot_target')
    cls_obj = CircleTarget() #instantiate object of the class (to call and use the functions)
    cls_obj.update_target()



if __name__ == '__main__':
    #this is where the script begins, calls the main function
    main()

