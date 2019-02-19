/*
 * Created By: Ihsan Olawale
 * Created On: February 16th, 2019
 * Description: Uses LaserScan data of a setup of cones to move Olaf the robot 
 *              through the gap in the cones
 */

#ifndef SAMPLE_PACKAGE_MYNODE_H
#define SAMPLE_PACKAGE_MYNODE_H

#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <sb_utils.h>

class GoThroughHole {
public:
    GoThroughHole(int argc, char **argv, std::string node_name);
    

private:
    /**
     * Callback function for when a LaserScan message is received
     *
     * @param msg the LaserScan data received in the callback
     */
    void subscriberCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
    

    /**
     * Function to determine gaps in the data


     * @param scanData the LaserScan data
     */
    std::vector<int> findGaps (sensor_msgs::LaserScan scan_data);

    /**
     * Function to find the hole in the cones
     *
     *
     * @param scan_data the LaserScan data
     *
     * @return the center of the hole relative to Olaf's origin
     */
    geometry_msgs::Point32 findHole (sensor_msgs::LaserScan scan_data); 

    /**
     * Function to convert LaserScan data to PointCloud with transformation
     *
     *
     * @param msg the LaserScan message to be transformed
     *
     * @return the PointCloud transformation of msg
     */
    sensor_msgs::PointCloud laserToPointCloud (sensor_msgs::LaserScan msg);

    /**
     * Function to bring Olaf to a halt, with the hopes of keeping the
     * transformation from LaserScan to PointCloud accurate
     *
     */
    void stopOlaf();

    /**
     * Function to angle Olaf towards the hole and inch forward
     *
     *
     * @param center the center of the hole
     */
    void moveToHole (geometry_msgs::Point32 center);

    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
