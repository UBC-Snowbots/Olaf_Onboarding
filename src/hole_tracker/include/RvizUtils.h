//
// Created by robyncastro on 10/11/17.
//

#ifndef HOLE_TRACKER_RVIZUTILS_H
#define HOLE_TRACKER_RVIZUTILS_H

// Messages
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace visualization_msgs;

class RvizUtils {
public:
    /**
     * Turn the points into a marker for rviz
     *
     * @param points the points to be converted
     * @param color the color of the points
     */
    static Marker displayPoints(vector<geometry_msgs::Point> points, Marker::_color_type color,
                                Marker::_scale_type scale, string frame_id, string ns);

    /**
     * Turn the point into a marker for rviz
     *
     * @param point the points to be converted
     * @param color the color of the point
     */
    static Marker displayPoint(geometry_msgs::Point point, Marker::_color_type color, Marker::_scale_type scale,
                               string frame_id, string ns);

    /**
     *
     */
    static Marker::_color_type createMarkerColor(float r, float g, float b, float a);

    /**
     *
     */
    static Marker::_scale_type createrMarkerScale(float x, float y, float z);
private:

    /**
     * Sets the properties of the marker (shape, color, header)
     *
     * @param marker
     * @param color
     */
    static void initialiseMarkerHeader(Marker &marker, string frame_id, string ns);

};


#endif //HOLE_TRACKER_RVIZUTILS_H
