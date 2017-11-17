//
// Created by robyncastro on 10/11/17.
//

#ifndef HOLE_TRACKER_RVIZUTILS_H
#define HOLE_TRACKER_RVIZUTILS_H

// Messages
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

using namespace std;
class RvizUtils {
public:
    /**
     * Turn the points into a marker for rviz
     *
     * @param points the points to be converted
     * @param color the color of the points
     */
    static visualization_msgs::Marker displayPoints(vector<geometry_msgs::Point> points, char color);

    /**
     * Turn the point into a marker for rviz
     *
     * @param point the points to be converted
     * @param color the color of the point
     */
    static visualization_msgs::Marker displayPoint(geometry_msgs::Point obstacle, char color);
private:

    /**
     * Sets the properties of the marker (shape, color, header)
     *
     * @param marker
     * @param color
     */
    static void initialiseMarkerParams(visualization_msgs::Marker &marker, char color);
};


#endif //HOLE_TRACKER_RVIZUTILS_H
