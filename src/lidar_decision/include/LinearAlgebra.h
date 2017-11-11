//
// Created by robyncastro on 10/11/17.
//

#ifndef LIDAR_LINEARALGEBRA_H
#define LIDAR_LINEARALGEBRA_H

// Messages
#include <geometry_msgs/Point.h>

using namespace std;

class LinearAlgebra {
public:
    LinearAlgebra();

    /**
     * Get the euclidean distance between two points
     *
     * @returns the euclidean distance
     */
    static double distanceBetweenPoints(geometry_msgs::Point point1, geometry_msgs::Point point2);

    /**
     * Get the point right in the middle of two points
     *
     * @returns the middle point
     */
    static geometry_msgs::Point getMiddlePoint(geometry_msgs::Point point1, geometry_msgs::Point point2);

    /**
     * Cross compare between two point groups and returns the closest pair
     *
     * @returns the closest pair
     */
    static pair<geometry_msgs::Point, geometry_msgs::Point> getClosestPair(vector<geometry_msgs::Point> points1,
                                                                           vector<geometry_msgs::Point> points2);

    /**
     * Determines the angle of the vector pointing to the point.
     *
     * @returns the angle to get to the point
     */
    static double getAngleToPoint(geometry_msgs::Point point);

private:
};

#endif //LIDAR_LINEARALGEBRA_H
