/*
 * Created By: Min Gyo Kim
 * Created On: November 25, 2017
 * Description: An example node that subscribes to obstacle avoider node
 */

#include <ObstacleAvoiderNode.h>

int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "obstacle_avoider_node";

    // Create an instance of your class
    ObstacleAvoiderNode my_class(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
