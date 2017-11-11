//
// Created by robyncastro on 10/11/17.
//

#ifndef LIDAR_LINEARALGEBRA_H
#define LIDAR_LINEARALGEBRA_H

#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace std;

class LinearAlgebra {
public:
    LinearAlgebra();

    static double distanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b);

    static geometry_msgs::Point getMiddlePoint(geometry_msgs::Point a, geometry_msgs::Point b);

    static pair<geometry_msgs::Point, geometry_msgs::Point> getClosestPair(vector<geometry_msgs::Point> points1,
                                                                           vector<geometry_msgs::Point> points2);

    static double getAngleToPoint(geometry_msgs::Point point);

private:
};

#endif //LIDAR_LINEARALGEBRA_H
