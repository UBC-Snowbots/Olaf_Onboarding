/*
 * Created By: Chris Heathe
 * Created On:  October28, 2017
 * Description: A node that will theoretically interpret lidar data,
 *              and help Olaf navigate through a hole in a row of cones
 *              Stress on *theoretically*
 */

#ifndef obstacle_avoidance_MYNODE_H
#define obstacle_avoidance_MYNODE_H

#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <geometry_msgs/Twist.h>

class MyClass {
public:
    MyClass(int argc, char **argv, std::string node_name);
    /**
     * Adds an exclamation point to a given string
     *
     * Some Longer explanation should go here
     *
     * @param input_string the string to add an exclamation point to
     *
     * @return input_string with an exclamation point added to it
     */
     static std::string addCharacterToString(std::string input_string, std::string suffix);
     std::string suffix;
    /**
     * Converts the target angle into geometric twist message
     *
     * @param target_angle the angle in radians towards the found hole
     * @return a twist message to move the robot towards the desired angle
     */
    static geometry_msgs::Twist targetAngletoTwist(float target_angle);


private:
    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
    void subscriberCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
    /**
     * Publishes a given string
     *
     * @param msg_to_publish the string to publish
     */
    void republishMsg(std::string msg_to_publish);
    void publishVel(geometry_msgs::Twist vel_to_publish);


    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
};
#endif //obstacle_avoidance_MYNODE_H
