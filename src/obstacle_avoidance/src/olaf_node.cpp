/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Modified By: Marinah Zhao
 * Modified On: Feb 12, 2018
 * Description: A node that subscribes to a lidar topic publishing twist messages for the robot to move.
 */

#include <OlafNode.h>


int main(int argc, char **argv){

    // Setup your ROS node
    std::string node_name = "olaf_node";

    // Create an instance of your class
    OlafClass olaf_node(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}