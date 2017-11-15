/*
 * Created By: Marcus Swift
 * Created On: October 20th, 2017
 * Description:Obstacle Avoidance Programming Challenge
 */

#include <OANode.h>

int main(int argc, char **argv) {
	
  ObstacleAvoidance avoid_obs(argc, argv, "obstacle_avoidance");
  ros::spin();

  return EXIT_SUCCESS;
}