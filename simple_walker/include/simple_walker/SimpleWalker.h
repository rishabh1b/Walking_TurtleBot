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
/**
* @brief Constructor for SimpleWalking algorithm turtlebot
* @param ros::NodeHandle node handle on which subscription and publishing would be made
*/
explicit SimpleWalker(ros::NodeHandle nh);
/**
* @brief processScan for processing the laser scan data as obtained from turtlebot
* @param ros::NodeHandle node handle on which subscription and publishing would be made
*/
void processScan(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
/**
* @brief private member variable velocity publisher
*/
ros::Publisher velPub_;
/**
* @brief private member variable Laser Scan Data Subscriber
*/
ros::Subscriber laserSub_;
/**
* @brief private member boolean to keep track of whether the obstacle is seen or not
*/
bool obstacleIsNear_;
/**
* @brief private member variable keeping track of exact velocity command to be published
*/
geometry_msgs::Twist msg;
};
