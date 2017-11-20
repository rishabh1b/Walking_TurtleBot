/**
* @file SimpleWalker.cpp Class Implementations to simply follow a path avoiding obstacles
* @author rishabh1b(Rishabh Biyani)
* @copyright MIT License (c) Rishabh Biyani 2017
*/

#include <vector>
#include "simple_walker/SimpleWalker.h"

#define SAFE_TOL 1 // 1 m distance as the tolerance

SimpleWalker::SimpleWalker(ros::NodeHandle nh) {
velPub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
laserSub_ = nh.subscribe("/scan", 10, &SimpleWalker::processScan, this);
obstacleIsNear_ = false;
// Start following Straight Line
msg.linear.x = 0;
msg.angular.z = 0.3;

velPub_.publish(msg);
}

void SimpleWalker::processScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
// ROS_INFO("Now Processing the Laser Scan Information");
std::vector<float> ranges = scan->ranges;
size_t sz = ranges.size();
int middleIndex = sz / 2;
float dist = ranges[middleIndex];
if (std::isnan(dist) || dist > scan->range_max || dist < scan->range_min) {
   // ROS_INFO("No Scan Messages received");
   velPub_.publish(msg);
   return;
}

if (dist < SAFE_TOL)
  obstacleIsNear_ = true;
else
  obstacleIsNear_ = false;

if (obstacleIsNear_) {
// Start Rotating in place
msg.linear.x = 0;
msg.angular.z = -0.3;
}
else {
// Follow Straight Line
msg.linear.x = 0.3;
msg.angular.z = 0;
}
velPub_.publish(msg);
}

int main(int argc, char* argv[]) {
ros::init(argc, argv, "simple_walker_node");
ros::NodeHandle n;
SimpleWalker simpleWalker(n);
ros::spin();
return 0;
}
