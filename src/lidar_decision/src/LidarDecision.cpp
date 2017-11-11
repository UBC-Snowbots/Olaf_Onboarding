/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Determines the twist message that aims for the hole
 *
 */
#include <LidarDecision.h>

using namespace std;

LidarDecision::LidarDecision(double angular_vel_cap, double linear_vel_cap, double angular_vel_multiplier,
                             double linear_vel_multiplier, double theta_scaling_multiplier, double max_distance_from_goal):
    angular_vel_cap(angular_vel_cap),
    linear_vel_cap(linear_vel_cap),
    angular_vel_multiplier(angular_vel_multiplier),
    linear_vel_multiplier(linear_vel_multiplier),
    theta_scaling_multiplier(theta_scaling_multiplier),
    max_distance_from_goal(max_distance_from_goal)
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
    geometry_msgs::Twist move_to_hole;
    geometry_msgs::Twist avoid_cones;
    initTwist(avoid_cones);
    initTwist(move_to_hole);

    // TODO: If merged points are too close move away from them

    double theta = LinearAlgebra().getAngleToPoint(hole);
    double distance = LinearAlgebra().distanceBetweenPoints(hole, origin);

    move_to_hole.angular.z = determineTurningVel(theta, distance);
    move_to_hole.linear.x = determineMovingVel(theta, distance);

    return move_to_hole;
}

double LidarDecision::determineTurningVel(double theta, double distance) {
    // TODO: Make turning speed dependant on distance to the hole
    double angular_vel = theta * theta_scaling_multiplier * angular_vel_multiplier;

    // Limit angular velocity
    if (fabs(angular_vel) > angular_vel_cap)
        angular_vel = angular_vel_cap * fabs(angular_vel) / angular_vel;

    return angular_vel;
}

double LidarDecision::determineMovingVel(double theta, double distance) {
    if (distance < max_distance_from_goal) return 0;

    double linear_vel = (distance * linear_vel_multiplier) / (theta * theta_scaling_multiplier);

    // Limit linear velocity
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