/*
 * Created By: Chris Heathe
 * Created On:  October 28, 2017
 * Description: A node that will theoretically interpret lidar data,
 *              and help Olaf navigate through a hole in a row of cones
 *              Stress on *theoretically*
 */
#include <ObstacleAvoidance.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "my_node";

    // Create an instance of your class
    ObstacleAvoidanceNode my_class(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}