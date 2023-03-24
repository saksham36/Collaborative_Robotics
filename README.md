# ME 326 Collaborative Robotics Stanford University

This repo is for getting started using Locobots in ME326 Collaborative Robotics Course at Stanford University taught by Prof. Monroe Kennedy ([Course Website](https://arm.stanford.edu/courses/me-326-collaborative-robotics))

Website of our project can be found here: [Project website](https://sites.google.com/stanford.edu/cs339-2023/home)

## Getting Started
If you are new to Linux ([how to install ubuntu/linux](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)) and ROS, the following are links you should start with once linux is installed: 
- Download ROS (ROS 1, noetic is tested in this example): http://wiki.ros.org/ROS/Installation
- [Download vim](https://www.tutorialspoint.com/vim/vim_installation_and_configuration.htm) to edit code/bash/etc in terminal: 
```
$ sudo apt-get update 
$ sudo apt-get install vim
```
- Install [net-tools](https://computingforgeeks.com/how-to-install-ifconfig-on-ubuntu-focal-fossa/) for *ifconfig* to check network: 
```
sudo apt install net-tools
``` 
- Install [Terminator](https://www.geeksforgeeks.org/terminator-a-linux-terminal-emulator/) for ease of terminal use (through preferences the banner can be removed):
```
sudo apt-get install terminator
```
- Install [python3](https://docs.python-guide.org/starting/install3/linux/)
```
$ sudo apt-get update
$ sudo apt-get install python3.6
```
- Install [scipy](https://scipy.org/install/)
```
sudo apt-get install python3-scipy
```
- Install [python-catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html): 
```
$ sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
 > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```
then
```
$ sudo apt-get update
$ sudo apt-get install python3-catkin-tools
```
refer to this link for the [quickstart](https://catkin-tools.readthedocs.io/en/latest/quick_start.html) and the [cheat sheet](https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html): https://catkin-tools.readthedocs.io/en/latest/index.html 

- Install PCL (Point Cloud Library) for ROS [pcl_ros](http://wiki.ros.org/pcl_ros).
```
$ sudo apt-get install ros-noetic-pcl-*
```

- To make file management easier for the package.xml and CMakeLists, this tutorial leverages the [helper code "catkin simple"](https://github.com/catkin/catkin_simple) when you make your package below you will import it into your workspace with `git clone https://github.com/catkin/catkin_simple.git`
- Become familiar with Github [github tutorials](https://docs.github.com/en/get-started/quickstart/hello-world), (learn with bitbucket tutorial, same methods, great graphics: [bitbucket tutorial](https://www.atlassian.com/git/tutorials/learn-git-with-bitbucket-cloud))

## Locobot Installation
On this page, follow these [instructions from Trossen Robotics](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros_interface/ros1/software_setup.html) under "Remote Install"

```
$ sudo apt install curl
$ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/xslocobot_remote_install.sh' > xslocobot_remote_install.sh
$chmod +x xslocobot_remote_install.sh
$./xslocobot_remote_install.sh -d noetic -b kobuki
```

Note: if you mess up while installing, just delete the interbotix_ws (from the level above the folder: `$ rm -rf interbotix_ws`) folder, then do the following in terminal `$ export ROS_IP=""`, then you can run the shell script again.

### Additional Changes required on Interbotix_ws to get the arm to work
- Delete all ```CATKIN_IGNORE``` in the ```interbotix_ws``` folder
- You'll need access to the LoCoBots in lab
```
sudo apt-get install ros-noetic-dynamixel-sdk
sudo apt-get install ros-noetic-rgbd*
sudo apt-get install ros-noetic-rplidar*
sudo apt-get install ros-noetic-joy

scp -r locobot@192.169.1.7:~/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions/meshes/locobot_meshes ~/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions/meshes/locobot_meshes

pip install modern_robotics
```


- If the gripper is not working, comment out line 444  of ```interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions/urdf/arms/mobile_wx250s.urdf.xacro```
The line in question requires the right finger to inversely "mimic" the left finger. When you send a command to both fingers it throws an error because the position command is over defined even if both position commands are equal and opposite (ie. 0.015 & -0.015).

## To run our code:
From the ```me326_locobot_example/launch/``` run ```launch_locobot_main.sh```
