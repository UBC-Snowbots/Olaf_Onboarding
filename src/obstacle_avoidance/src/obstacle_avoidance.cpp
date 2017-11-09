/*
 * Created By: Marcus Swift
 * Created On: October 20th, 2017
 * Description:Obstacle Avoidance Programming Challenge
 */

#include <MyNode.h>


int main(int argc, char **argv){
    AvoidObstacleClass avoid_obs(argc, argv, "obstacle_avoidance");
    // Start up ros. This will continue to run until the node is killed
    //ros::spin();

    // Once the node stops, return 0
	
	double ang_velocity = 0;
	bool started_on_left = avoid_obs.isLeftSideOfGoal();
	bool first_action = TRUE;
	bool first_period_no_obs = TRUE;
	
	ros::Rate loop_rate(1000);
	while (ros::ok()) {
		ang_velocity = avoid_obs.angVelocityToAvoidCollision();
		//std::cout << "ang Vel " << ang_velocity << " " << std::endl; 
		if ((ang_velocity != 0) && (first_action == TRUE)){
			first_period_no_obs = FALSE;
			if (ang_velocity > 0) {
				started_on_left = FALSE;
			} else if (ang_velocity <= 0) {
				started_on_left = TRUE;
			}
		} else if ((ang_velocity == 0) && (first_period_no_obs == FALSE)) {
			first_action = FALSE;
			if (started_on_left == FALSE) {
				ang_velocity = -WALL_HUG_VEL;
			} else {
				ang_velocity = WALL_HUG_VEL;
			}
		}
		avoid_obs.setAngVelMsg(ang_velocity);
		avoid_obs.publishVel();
		std::cout << "1st action? " << first_action << " 1st no obs? " << first_period_no_obs 
		 << " start left? " << started_on_left << " ang velocity " << ang_velocity << std::endl;
		//std::cout << " start left? " << started_on_left << std::endl;
		ros::spinOnce();
        loop_rate.sleep();
	}
	/*
	while (ros::ok()) {
		ang_velocity = avoid_obs.angVelocityToAvoidCollision();
		//std::cout << "ang Vel " << ang_velocity << " " << std::endl; 
		if ((ang_velocity != 0) && (first_action == TRUE)){
			first_period_no_obs = FALSE;
			if ((started_on_left == TRUE) && (ang_velocity > 0)) {
				ang_velocity = ang_velocity*-1;
			} else if ((started_on_left == FALSE) && (ang_velocity < 0)) {
				ang_velocity = ang_velocity*-1;
			}
		} else if ((ang_velocity == 0) && (first_period_no_obs == FALSE)) {
			first_action = FALSE;
			if (started_on_left == FALSE) {
				ang_velocity = WALL_HUG_VEL;
			} else {
				ang_velocity = -WALL_HUG_VEL;
			}
		}
		avoid_obs.setAngVelMsg(ang_velocity);
		avoid_obs.publishVel();
		std::cout << "1st action? " << first_action << " 1st no obs? " << first_period_no_obs 
		 << " start left? " << started_on_left << " ang velocity " << ang_velocity << std::endl;
		//std::cout << " start left? " << started_on_left << std::endl;
		ros::spinOnce();
        loop_rate.sleep();
	} */
		avoid_obs.setLinearVelMsg(0);
		avoid_obs.setAngVelMsg(0);
		avoid_obs.publishVel();
		ros::spinOnce();
        loop_rate.sleep();
	
   


    return EXIT_SUCCESS;
}