//
// Created by robyncastro on 10/11/17.
//

#ifndef LIDAR_RVIZUTILS_H
#define LIDAR_RVIZUTILS_H

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

using namespace std;
class RvizUtils {
public:
    static visualization_msgs::Marker displayPoints(vector<geometry_msgs::Point> points, char color);
    static visualization_msgs::Marker displayPoint(geometry_msgs::Point obstacle, char color);
private:
    static void initialiseMarkerParams(visualization_msgs::Marker &marker, char color);
};


#endif //LIDAR_RVIZUTILS_H
