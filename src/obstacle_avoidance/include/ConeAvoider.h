/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Modified By: Marinah Zhao
 * Modified On: Feb 12, 2018
 * Description: A node that subscribes to a lidar topic publishing twist messages for the robot to move.
 */

#ifndef OBSTACLE_AVOIDANCE_CONEAVOIDER_H
#define OBSTACLE_AVOIDANCE_CONEAVOIDER_H

#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

class ConeAvoider {
public:
    ConeAvoider(int argc, char **argv, std::string node_name);

    /**
     * Callback function for when a new laser scan is received
     *
     * @param scan the laser scan received in the callback
     */
    void lidarSubscriberCallBack(const sensor_msgs::LaserScan scan);


private:
    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
};
#endif //OBSTACLE_AVOIDANCE_CONEAVOIDER_H_H
