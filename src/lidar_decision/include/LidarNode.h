//
// Created by robyncastro on 10/11/17.
//

#ifndef LIDAR_LIDARNODE_H
#define LIDAR_LIDARNODE_H

// Messages
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

// Lidar Specific
#include <LidarDecision.h>
#include <LidarObstacleManager.h>

// ROS
#include <ros/ros.h>

// Utilities
#include <sb_utils.h>
#include <RvizUtils.h>

using namespace std;
class LidarNode {
public:
    LidarNode(int argc, char **argv, string node_name);
private:
    ros::Subscriber laser_scan_subscriber;

    // Motion Publisher
    ros::Publisher twist_publisher;

    // Debug Publishers
    ros::Publisher cone1_debug_publisher;
    ros::Publisher cone2_debug_publisher;
    ros::Publisher hole_debug_publisher;

    // Controller
    LidarDecision lidar_decision;

    // Rviz
    RvizUtils rviz_utils;

    /**
     * Callback function for when a new laser scan is received
     *
     * @param laser_scan the laser scan received in the callback
     */
    void laserScanCallBack(const sensor_msgs::LaserScan laser_scan);

    /**
     * Publishes the twist
     *
     * @param msg_to_publish the string to publish
     */
    void publishTwist(geometry_msgs::Twist twist_msg);

    /**
     * Setup the decision parameters
     *
     * @param private_nh the private node handle
     */
    void initDecisionParams(ros::NodeHandle private_nh);

    /**
     * Setup the obstacle manager parameters
     *
     * @param private_nh the private node handle
     */
    void initObstacleManagerParams(ros::NodeHandle private_nh);

    /**
     * Setup the subscribers
     */
    void initSubscribers(ros::NodeHandle nh);

    /**
     * Setup the publishers
     */
    void initPublishers(ros::NodeHandle private_nh);

    /**
     * Set
     */

    // Decision parameters
    double angular_vel_cap;
    double linear_vel_cap;
    double linear_vel_multiplier;
    double angular_vel_multiplier;
    double theta_scaling_multiplier;

    // Obstacle Manager parameters
    double cone_grouping_tolerance;
    double max_scan_distance;

};


#endif //LIDAR_LIDARNODE_H
