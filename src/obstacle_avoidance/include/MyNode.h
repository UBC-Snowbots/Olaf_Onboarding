#ifndef OBSTACLE_AVOIDANCE_MYNODE_H
#define OBSTACLE_AVOIDANCE_MYNODE_H

#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class MyClass {
public:
    MyClass(int argc, char **argv, std::string node_name);


private:
    void solution();
    void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan_message);
    ros::Subscriber laser_subscriber;
    ros::Publisher velocity_publisher;

    sensor_msgs::LaserScan laser_message;
    geometry_msgs::Twist vel_msg;
};
#endif //OBSTACLE_AVOIDANCE_MYNODE_H
