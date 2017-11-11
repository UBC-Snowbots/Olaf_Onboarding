//
// Created by robyncastro on 28/10/17.
//

#ifndef OLAF_ONBOARDING_LIDARDECISION_H
#define OLAF_ONBOARDING_LIDARDECISION_H

#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <LidarObstacleManager.h>
#include <LinearAlgebra.h>

using namespace std;

class LidarDecision {
public:
    LidarDecision(double angular_vel_cap, double linear_vel_cap, double angular_vel_multiplier,
                  double linear_vel_multiplier, double theta_scaling_multiplier);

    LidarDecision();

    geometry_msgs::Twist determineDesiredMotion(vector<vector<geometry_msgs::Point>> merged_points,
                                                geometry_msgs::Point hole);

    double determineTurningVel(double theta, double distance);
    double determineMovingVel(double angular_vel);
    void initTwist(geometry_msgs::Twist &twist);

private:
    geometry_msgs::Point origin;

    double angular_vel_cap;
    double linear_vel_cap;
    double linear_vel_multiplier;
    double angular_vel_multiplier;
    double theta_scaling_multiplier;
};

#endif //OLAF_ONBOARDING_LIDARDECISION_H
