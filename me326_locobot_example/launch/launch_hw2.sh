#!/bin/bash
gnome-terminal -x roslaunch me326_locobot_example hw2.launch 
sleep 10
rosservice call /gazebo/unpause_physics


