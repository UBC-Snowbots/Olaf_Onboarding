//
// Created by robyncastro on 10/11/17.
//
#include <LinearAlgebra.h>

LinearAlgebra::LinearAlgebra() {
    // Empty Constructor
}

double LinearAlgebra::distanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b) {
    return sqrt(pow(b.y - a.y, 2) + pow(b.x - a.x, 2));
}

geometry_msgs::Point LinearAlgebra::getMiddlePoint(geometry_msgs::Point a, geometry_msgs::Point b) {
    geometry_msgs::Point hole;
    hole.x = (a.x + b.x) / 2;
    hole.y = (a.y + b.y) / 2;

    return hole;
}

pair<geometry_msgs::Point, geometry_msgs::Point> LinearAlgebra::getClosestPair(
        vector<geometry_msgs::Point> points1, vector<geometry_msgs::Point> points2) {
    pair<geometry_msgs::Point, geometry_msgs::Point> closest_pair;

    double closest_distance = distanceBetweenPoints(points1[0], points2[0]);
    closest_pair.first = points1[0];
    closest_pair.second = points2[0];

    for (int i = 0; i < points1.size(); i++) {
        for (int j = 0; j < points2.size(); j++) {
            double cur_distance = distanceBetweenPoints(points1[i], points2[j]);
            if (cur_distance < closest_distance) {
                closest_distance = cur_distance;
                closest_pair.first = points1[i];
                closest_pair.second = points2[j];
            }
        }
    }

    return closest_pair;
}

double LinearAlgebra::getAngleToPoint(geometry_msgs::Point point) {
    if (point.x == 0)
        return 0;
    else
        return atan(point.y / point.x);
}