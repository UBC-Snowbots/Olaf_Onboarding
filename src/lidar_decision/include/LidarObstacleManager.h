//
// Created by robyncastro on 10/11/17.
//

#ifndef LIDAR_LIDAROBSTACLEMANAGER_H
#define LIDAR_LIDAROBSTACLEMANAGER_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <LinearAlgebra.h>

using namespace std;

class LidarObstacleManager {
public:
    LidarObstacleManager(sensor_msgs::LaserScan);

    vector<geometry_msgs::Point> getPoints();

    vector<vector<geometry_msgs::Point>> getMergedPoints();

    geometry_msgs::Point getHole();

private:
    geometry_msgs::Point hole;
    vector<geometry_msgs::Point> points;
    vector<vector<geometry_msgs::Point>> merged_points;

    /**
     * Construct a vector of points given a laser scan
     *
     * @param laser_scan the laser scan to be parsed
     */
    vector<geometry_msgs::Point> constructPoints(sensor_msgs::LaserScan laser_scan);

    vector<vector<geometry_msgs::Point>> mergePoints();

    bool findMatch(vector<vector<geometry_msgs::Point>> &merged_points, geometry_msgs::Point point);
};

#endif //LIDAR_LIDAROBSTACLEMANAGER_H
