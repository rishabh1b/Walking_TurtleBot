// MIT License

// Copyright (c) 2017 Rishabh Biyani

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
