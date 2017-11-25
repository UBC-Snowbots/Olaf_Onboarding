/*
 * Created By: Min Gyo Kim
 * Created On: November 25, 2017
 * Description: Header file for ObstacleAvoiderNode
 */

#ifndef OBSTACLE_AVOIDER_NODE_H
#define OBSTACLE_AVOIDER_NODE_H

#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ObstacleAvoider.h>
#include <string>

class ObstacleAvoiderNode {
public:
    ObstacleAvoiderNode(int argc, char **argv, std::string node_name);

    // main function
    void goThoughCones();


private:
    ObstacleAvoider obstacleAvoider;

    float _forward_vel;  // in m/s

    struct ObstacleAvoider::angleInfo _angle_info;

    struct ObstacleAvoider::rangeInfo _range_info;

    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
    void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;

    std::vector<float> _ranges;
};
#endif //OBSTACLE_AVOIDER_NODE_H
