/**
* @file SimpleWalker.h Class defnitions to simply follow a path avoiding obstacles
* @author rishabh1b(Rishabh Biyani)
* @copyright MIT License (c) Rishabh Biyani 2017
*/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class SimpleWalker {

public:
SimpleWalker();
void processScan(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
ros::NodeHandle nh_;
ros::Publisher velPub_;
ros::Subscriber laserSub_;
bool obstacleIsNear_;
};
