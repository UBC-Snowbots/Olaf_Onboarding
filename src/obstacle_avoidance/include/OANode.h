/*
 * Created By: Marcus Swift
 * Created On: October 20th, 2017
 * Description: A node that subscribes to the laser scan message and outputs
 * a twist message that allows the vehicle to avoid obstacles
 */

#ifndef OANODE_H
#define OANODE_H

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <sb_utils.h>
#include <std_msgs/String.h>

class ObstacleAvoidance {
	
private:
  enum avoidance { left, right, no_avoidance };
  //constants
  const float vehicle_width = 0.35;      // metres
  const float min_critical_distance = 2; // metres
  const float ang_vel_gain = 1.2;
  const float wall_hug_vel = 0.8;
  const float linear_vel = 0.35;
  const float min_goal_width = 0.32;
  const float max_goal_width = 0.6;

  ros::Publisher twist_publisher;
  ros::Subscriber laser_scan_subscriber;
  geometry_msgs::Twist vel_msg;
  sensor_msgs::LaserScan lidar_data;

public:
  
  //Class Functions
  
  /**
     * Constructs the obstacle avoidance nide
     *
     * @param arguments passed into main and the string of the node name
  */
  ObstacleAvoidance(int argc, char **argv, std::string node_name);

  /**
     * Default Constructor
  */
  ObstacleAvoidance();

/**
     * Destructor
  */
  ~ObstacleAvoidance();
  
  /**
     * Determines what direction the vehicle needs to turn in order
     * to be able to avoid the obstacle. Returns no_avoidance when
     * no action needs to be taken
     *
     * @param distance in metres found in ranges vector in scan_msg
     * and the angle in radians that the lidar sees that distance
	 * 
     * @return avoidance enumerator left, right or no_avoidance
  */
  avoidance moveToAvoidCollision(float lidar_distance, float lidar_angle);

  /**
     * Returns the angular velocity required in order to avoid the
     * closest obstacle
	 * 
     * @return angular velocity in arbritary units that reflect steering
	 * postion of the vehicle
  */
  double angVelocityToAvoidCollision();

  /**
     * publishes a velocity message to the cmd_vel topic from vel_msg
  */
  void publishVel();

  /**
     * Sets the value of vel_msg.linear.x
	 * 
     * @param In arbritary units related to the vehicle
  */
  void setLinearVelMsg(float linear_vel);

  /**
     * Sets the value of vel_msg.angular.z
	 * 
     * @param In arbritary units that reflect steering
	 * postion of the vehicle
  */
  void setAngVelMsg(float ang_vel);
/**
     * Gets the value of vel_msg.linear.x
	 * 
     * @return In arbritary units related to the vehicle
  */
  float getLinearVelMsg();

  /**
     * Gets the value of vel_msg.angular.z
	 * 
     * @return In arbritary units that reflect steering
	 * postion of the vehicle
  */
  float getAngVelMsg();
  
  /**
     * The main logic of the obstacle_avoidance node
	 * 
     * This function gets the data from the subscribed topic and determines
	 * when to publish angular velocities to turn into the wall and when
	 * to avoid it
     *
     * @param pointer to the subscribed topic data
  */
  void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_message);


};
#endif // OANODE_H
