/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef SAMPLE_PACKAGE_MYNODE_H
#define SAMPLE_PACKAGE_MYNODE_H

#include <iostream>
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
