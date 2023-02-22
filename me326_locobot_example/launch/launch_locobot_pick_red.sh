#!/bin/bash

# Launch Gazebo and init env
gnome-terminal -x roslaunch interbotix_xslocobot_moveit xslocobot_moveit.launch robot_model:=locobot_wx250s show_lidar:=true use_gazebo:=true dof:=6
sleep 10
rosservice call /gazebo/unpause_physics

# Spawn new blocks
sleep 5
roslaunch me326_locobot_example random_cube_spawn.launch

# Launch point cloud
sleep 5
gnome-terminal -x rosrun me326_locobot_example matching_ptcld_serv


# Launch Mocap Simulator
sleep 5
gnome-terminal -x roslaunch me326_locobot_example optitrack_simulator.launch

# Code file
gnome-terminal -x roslaunch me326_locobot_example gazebo_moveit_example.launch 
# Launch Locobot pick and place
#gnome-terminal -x roslaunch me326_locobot_example locobot_pick_red.launch


# Launch apriltag spawn and detector
#mkdir -p ~/.gazebo/models/
#cp -R ~/me326_ws/src/collaborative_robotics_course/me326_locobot_example/model/Ap* ~/.gazebo/models/
#sleep 5
#roslaunch me326_locobot_example apriltag_spawn.launch
#roslaunch me326_locobot_example apriltag_detector.launch

