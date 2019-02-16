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
#include <vector>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <sb_utils.h>

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
    // static std::string addCharacterToString(std::string input_string, std::string suffix);
     // std::string suffix;

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
    // void republishMsg(std::string msg_to_publish);

    /**
     * Function to determine gaps in the data


     * @param scanData the LaserScan data
     */
    std::vector<int> findGaps (sensor_msgs::LaserScan scanData);

    geometry_msgs::Point32 findHole (sensor_msgs::PointCloud scanData); 
    void moveThroughHole (geometry_msgs::Point32 centerOfHole);
    void findHole (sensor_msgs::PointCloud msg);
    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
