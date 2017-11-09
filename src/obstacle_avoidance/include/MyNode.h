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
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#define LIDAR_ARRAY_SIZE 100
#define VEHICLE_WIDTH 0.35 //metres
#define MIN_CRITICAL_DISTANCE 2 //metres
#define ANG_VEL_GAIN 1.2
#define WALL_HUG_VEL 0.8
#define LINEAR_VEL 0.35
#define MIN_GOAL_WIDTH 0.32
#define MAX_GOAL_WIDTH 0.6
#define TRUE 1
#define FALSE 0

enum avoidance {
	left,
	right,
	no_avoidance
};

class AvoidObstacleClass {
public:
	
    AvoidObstacleClass(int argc, char **argv, std::string node_name);
	
	AvoidObstacleClass();
	
	~AvoidObstacleClass();
	
	avoidance moveToAvoidCollision(float lidar_distance, float lidar_angle);
	 
	bool isLeftSideOfGoal();
	
	double angVelocityToAvoidCollision();
	
	void publishVel();
	
	void setLinearVelMsg(float linear_vel);

	void setAngVelMsg(float ang_vel);
	
	float getLinearVelMsg();
	
	float getAngVelMsg();
	
	void lidarCallback(const sensor_msgs::LaserScan::ConstPtr & laser_scan_message);
	
private:
	
	ros::NodeHandle *nh;
    ros::Publisher twist_publisher;
	ros::Subscriber laser_scan_subscriber;
	static sensor_msgs::LaserScan lidar_data;
	geometry_msgs::Twist vel_msg;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
