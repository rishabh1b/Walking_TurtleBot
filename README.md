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
Install using -
```
sudo apt-get install ros-kinetic-turtlebot-gazebo
```

3. CMake. This can be downloaded from [here](https://cmake.org/download/)

## Basic Demonstration
Following commands will spawn turtlebot in Gazebo environment (turtlebot_world) to simulate an obstacle avoidance behaviour using laserscan data
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch simple_walker demo.launch
```
## Working with Bag Files
For optionally recording the bag files with all the topics except ```/camera/*``` topics for a specific duration run the following
```
roslaunch beginner_tutorials demo.launch rec_bag:=true bag_dur:=30
```
The bag file get saved in ```~/.ros``` folder 

Bag Files can be inspected using
```
rosbag info bagfile.bag
```
This will enlist all the topics recorded.

#### For testing the recorded bag file run the following commands in separate terminals- 
```
roscore
```
```
cd ~/catkin_ws/src/simple_walker
rosrun rviz rviz -d bagTest.rviz
```
```
cd ~/catkin_ws/src/simple_walker/outputs/bagFiles
rosbag play bagFileName.bag
```
This will simulate turtlebot's ```base_footprint``` frame with reference to ```odom``` frame and also show laser scan readings by default in rviz.
