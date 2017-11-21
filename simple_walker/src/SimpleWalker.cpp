/**
* @file SimpleWalker.cpp Class Implementations to simply follow a path avoiding obstacles
* @author rishabh1b(Rishabh Biyani)
* @copyright MIT License (c) Rishabh Biyani 2017
*/

#include <math.h>
#include <vector>
#include "simple_walker/SimpleWalker.h"

#define SAFE_TOL 1.2 // 1.2m distance as the tolerance
#define SAFE_ANGLE 12 // 12 degrees

SimpleWalker::SimpleWalker(ros::NodeHandle nh) {
velPub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
laserSub_ = nh.subscribe("/scan", 10, &SimpleWalker::processScan, this);
obstacleIsNear_ = false;
// Start Rotating in Place
msg.linear.x = 0;
msg.angular.z = 0.3;

velPub_.publish(msg);
}

void SimpleWalker::processScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
// ROS_INFO("Now Processing the Laser Scan Information");

// Get the Ranges from the Laser Scan
std::vector<float> ranges = scan->ranges;
size_t sz = ranges.size();
int middleIndex = sz / 2;

// Check about SAFE_ANGLE degrees of laser scan readings from the zero degree
float angle_check = SAFE_ANGLE * M_PI / 180;
float increment = scan->angle_increment;
int range = round(angle_check / increment);
size_t min_range = middleIndex - range;
size_t max_range = middleIndex + range;

// Check for the range of values to see whether there could be a collision
float min_dist;
min_dist = ranges[min_range];
for (auto i = min_range; i < max_range; i++) {
if (ranges[i] < min_dist)
   min_dist = ranges[i];
}

// If any scanner readings are not available,
//  we rotate in place to ensure that we remain bounded near to obstacles
if (std::isnan(min_dist) || min_dist > scan->range_max || min_dist < scan->range_min) {
   // ROS_INFO("No Scan Messages received");
   velPub_.publish(msg);
   return;
}

if (min_dist < SAFE_TOL)
  obstacleIsNear_ = true;
else
  obstacleIsNear_ = false;

if (obstacleIsNear_) {
// Rotate in Place
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

/**
* @brief main() function for initializing the node and pushing the callbacks
* @return 0 if process exits smoothly
*/
int main(int argc, char* argv[]) {
ros::init(argc, argv, "simple_walker_node");
ros::NodeHandle n;
SimpleWalker simpleWalker(n);
ros::spin();
return 0;
}
