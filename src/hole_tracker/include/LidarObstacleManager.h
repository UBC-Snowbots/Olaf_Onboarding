//
// Created by robyncastro on 10/11/17.
//

#ifndef HOLE_TRACKER_LIDAROBSTACLEMANAGER_H
#define HOLE_TRACKER_LIDAROBSTACLEMANAGER_H

// Messages
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// Utilities
#include <LinearAlgebra.h>

class LidarObstacleManager {
public:
    // Required empty constructor.
    LidarObstacleManager();

    /**
     *  Sets up the obstacle manager.
     */
    LidarObstacleManager(double max_scan_distance, double cone_grouping_tolerance);

    /**
     *  Takes in a laser scan and initialises this LidarObstacleManager's obstacles, merged obstacles, and hole.
     *
     *  @param laser_scan
     */
    void parseLaserScan(sensor_msgs::LaserScan laser_scan);

    /**
     *  Returns all the obstacles inside the laser scan.
     *
     *  @return obstacles
     */
    std::vector<geometry_msgs::Point> getObstacles();

    /**
     *  Return groups of obstacles that were grouped together based on proximity.
     *
     *  @return obstacle groups
     */
    std::vector<std::vector<geometry_msgs::Point>> getMergedObstacles();

    /**
     *  Return where the hole in the wall is.
     *
     *  @return hole in the wall
     */
    geometry_msgs::Point getHole();

private:
    /**
     *  Construct a std::vector of valid obstacles given a laser scan
     *
     *  @param laser_scan the laser scan to be parsed
     *  @return a std::vector of valid obstacles
     */
    std::vector<geometry_msgs::Point> constructObstacles(sensor_msgs::LaserScan laser_scan);

    /**
     *  Check if a obstacle is valid
     *
     *  @param obstacle the obstacle to be checked
     *  @param range_max the furthest the obstacle can be to be considered valid
     *  @param range_min the closest the obstacle has to be to be considered valid
     *  @return obstacle validity
     */
    bool validateObstacle(float range, float range_max, float range_min);

    /**
     *  Convert polar point to cartesian point
     *
     *  @param range the distance from origin
     *  @param theta the angle to x-axis
     *
     *  @return the cartesian point
     */
    geometry_msgs::Point polarToCartesian(float range, float theta);

    /**
     *  Group obstacles based on proximity
     *
     *  @param obstacles the obstacles to be merged
     *
     *  @return obstacle groups
     */
    std::vector<std::vector<geometry_msgs::Point>> mergeObstacles(std::vector<geometry_msgs::Point> obstacles);

    /**
     *  Groups the obstacle into currently existing merged obstacles
     *  
     *  @param merged_obstacles the currently existing merged obstacles
     *  @param obstacle the obstacle to be merged in
     *  
     *  @returns whether or not it grouped the obstacle
     */
    bool findMatch(std::vector<std::vector<geometry_msgs::Point>> &merged_obstacles, geometry_msgs::Point obstacle);

    // Obstacles
    std::vector<geometry_msgs::Point> obstacles;
    std::vector<std::vector<geometry_msgs::Point>> merged_obstacles;

    // The hole in the wall
    geometry_msgs::Point hole;

    // Obstacle Manager parameters
    double cone_grouping_tolerance;
    double max_scan_distance;
};

#endif //HOLE_TRACKER_LIDAROBSTACLEMANAGER_H
