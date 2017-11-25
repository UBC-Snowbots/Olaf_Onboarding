/*
 * Created By: Chris Heathe
 * Created On:  October28, 2017
 * Description: A node that interprets lidar data,
 *              and directs Olaf to navigate through a hole in a row of cones
 */

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <geometry_msgs/Twist.h>

class ObstacleAvoidanceNode {
public:
    ObstacleAvoidanceNode(int argc, char **argv, std::string node_name);
    /**
     * Adds an exclamation point to a given string
     *
     * Some Longer explanation should go here
     *
     * @param input_string the string to add an exclamation point to
     *
     * @return input_string with an exclamation point added to it
     */
    static geometry_msgs::Twist targetAngletoTwist(float target_angle);


private:
    /**
     * Callback function for when a new laserscan is received
     *
     * @param msg the laserscan received in the callback
     */
    void subscriberCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
    /**
     * Publishes a given Twist
     *
     * @param vel_to_publish the twist to publish
     */
    void publishVel(geometry_msgs::Twist vel_to_publish);

    /**
     * Determines a target angle from a laserscan
     *
     * @param msg the laserscan is received when called
     *
     * @return targetAngle of Olaf trajectory
     */
    float calculateTargetAngle(const sensor_msgs::LaserScan::ConstPtr &msg);

    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
};
#endif //OBSTACLE_AVOIDANCE_H
