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
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ObstacleAvoider.h>
#include <string>

#define RAD2DEG(x) ((x)*180./M_PI)
// #define OLAF_WIDTH 0.5  // 0.5 m?
// #define FORWARD_VELOCITY 1  // 1 m/s?

class MyClass {
public:
    MyClass(int argc, char **argv, std::string node_name);

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
#endif //SAMPLE_PACKAGE_MYNODE_H
