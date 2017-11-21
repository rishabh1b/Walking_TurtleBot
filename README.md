# Basic Obstacle Avoidance with TurtleBot
This repository implements a naive method to avoid obstacles and keep moving in a straight line. 

## Standard install via command-line
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/rishabh1b/Walking_TurtleBot/
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```

## Dependencies
1. ROS Kinetic. This can be downloaded by following the steps [here](http://wiki.ros.org/kinetic/Installation).
2. ROS standard package for TurtleBot Simulation in Gazebo- ``` ros-kinetic-turtlebot-gazebo ```
2. CMake. This can be downloaded from [here](https://cmake.org/download/)

## Basic Demonstration
Following commands will spawn turtlebot in Gazebo environment (turtlebot_world) to simulate a obstacle avoidance behaviour using laserscan data
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch simple_walker demo.launch
```
