/*
 * Created By: John Shin
 * Created On: Oct 29, 2019
 * Description: A node that initiates obstacle avoidance node
 */

#include <ObstacleAvoidance.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "obstacle_avoidance";

    // Create an instance of your class
    ObstacleAvoidance obstacle_avoidance(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
