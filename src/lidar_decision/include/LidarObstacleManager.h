//
// Created by robyncastro on 10/11/17.
//

#ifndef LIDAR_LIDAROBSTACLEMANAGER_H
#define LIDAR_LIDAROBSTACLEMANAGER_H

// Messages
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// Utilities
#include <LinearAlgebra.h>

using namespace std;

class LidarObstacleManager {
public:
    LidarObstacleManager(sensor_msgs::LaserScan, double max_scan_distance, double cone_grouping_tolerance);

    // Getter functions
    vector<geometry_msgs::Point> getPoints();
    vector<vector<geometry_msgs::Point>> getMergedPoints();
    geometry_msgs::Point getHole();

private:
    /**
     * Construct a vector of valid obstacles given a laser scan
     *
     * @param laser_scan the laser scan to be parsed
     * @returns a vector of valid obstacles
     */
    vector<geometry_msgs::Point> constructPoints(sensor_msgs::LaserScan laser_scan);

    /**
     * Check if a point is valid
     *
     * @param point the point to be checked
     * @returns point validity
     */
    bool validatePoint(float range, float range_max, float range_min);

    /**
     * Convert polar point to cartesian point
     *
     * @returns the cartesian point
     */
    geometry_msgs::Point polarToCartesian(float range, float theta);

    /**
     * Group points based on proximity
     *
     * @returns a vector of grouped points
     */
    vector<vector<geometry_msgs::Point>> mergePoints(vector<geometry_msgs::Point> points);

    /**
     * Groups the point into currently existing merged points
     *
     * @returns whether or not it grouped the point
     */
    bool findMatch(vector<vector<geometry_msgs::Point>> &merged_points, geometry_msgs::Point point);

    // Obstacles
    vector<geometry_msgs::Point> points;
    vector<vector<geometry_msgs::Point>> merged_points;

    // The hole in the obstacles
    geometry_msgs::Point hole;

    // Obstacle Manager parameters
    double cone_grouping_tolerance;
    double max_scan_distance;
};

#endif //LIDAR_LIDAROBSTACLEMANAGER_H
