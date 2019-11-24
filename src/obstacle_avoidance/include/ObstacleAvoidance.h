/*
 * Created By: John Shin
 * Created On: October 29, 2019
 * Description: a node that guides Olaf to go though a hole
 */

#ifndef SAMPLE_PACKAGE_MYNODE_H
#define SAMPLE_PACKAGE_MYNODE_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sb_utils.h>

class ObstacleAvoidance {
public:
    ObstacleAvoidance(int argc, char **argv, std::string node_name);

private:
    void subscriberCallBack(const sensor_msgs::LaserScan::ConstPtr &laser_scan);

    void republishMsg(geometry_msgs::Twist twist_msg);

    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
};
#endif 
