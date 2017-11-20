/**
* @file SimpleWalker.cpp Class Implementations to simply follow a path avoiding obstacles
* @author rishabh1b(Rishabh Biyani)
* @copyright MIT License (c) Rishabh Biyani 2017
*/


#include "simple_walker/SimpleWalker.h"

SimpleWalker::SimpleWalker() {
velPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
laserSub_ = nh_.subscribe("/scan", 1, &SimpleWalker::processScan, this);
obstacleIsNear_ = false;
}

void SimpleWalker::processScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
return;
}

int main(int argc, char* argv[]) {
ros::init(argc, argv, "simple_walker_node");
SimpleWalker simpleWalker();
ros::spin();
return 0;
}
