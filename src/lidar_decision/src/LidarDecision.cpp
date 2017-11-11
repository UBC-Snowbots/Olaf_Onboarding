/*
 * Created By: Robyn Castro
 * Created On: July 16th, 2016
 * Description: A node that takes in a laser scan, and publishes
 *              a twist message.
 */

#include <LidarDecision.h>

using namespace std;

LidarDecision::LidarDecision(double angular_vel_cap, double linear_vel_cap, double angular_vel_multiplier,
                             double linear_vel_multiplier, double theta_scaling_multiplier):
    angular_vel_cap(angular_vel_cap),
    linear_vel_cap(linear_vel_cap),
    angular_vel_multiplier(angular_vel_multiplier),
    linear_vel_multiplier(linear_vel_multiplier),
    theta_scaling_multiplier(theta_scaling_multiplier)
{
    // Setup origin point
    origin.x = 0;
    origin.y = 0;
}

LidarDecision::LidarDecision() {
    // Setup origin point
    origin.x = 0;
    origin.y = 0;
}

geometry_msgs::Twist LidarDecision::determineDesiredMotion(vector<vector<geometry_msgs::Point>> merged_points,
                                                           geometry_msgs::Point hole) {
    geometry_msgs::Twist follow_hole;
    initTwist(follow_hole);

    double theta = LinearAlgebra().getAngleToPoint(hole);
    double distance = LinearAlgebra().distanceBetweenPoints(hole, origin);

    double angular_vel = determineTurningVel(theta, distance);
    follow_hole.angular.z = angular_vel;
    follow_hole.linear.x = determineMovingVel(angular_vel);

    return follow_hole;
}

double LidarDecision::determineTurningVel(double theta, double distance) {
    // Figure out how fast we should be turning
    double angular_vel = (theta_scaling_multiplier * theta + distance) * angular_vel_multiplier;

    if (fabs(angular_vel) > angular_vel_cap)
        angular_vel = angular_vel_cap * fabs(angular_vel) / angular_vel;

    return angular_vel;
}

double LidarDecision::determineMovingVel(double angular_vel) {
    if (angular_vel == 0) return linear_vel_cap;

    double linear_vel = 1 / fabs(angular_vel);

    if (fabs(linear_vel) > linear_vel_cap)
        linear_vel = linear_vel_cap * fabs(linear_vel) / linear_vel;

    return linear_vel;
}

void LidarDecision::initTwist(geometry_msgs::Twist &twist) {
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
}