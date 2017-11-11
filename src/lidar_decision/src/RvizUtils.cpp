//
// Created by robyncastro on 10/11/17.
//

#include <RvizUtils.h>

visualization_msgs::Marker RvizUtils::displayPoints(vector<geometry_msgs::Point> obstacles, char color) {
    visualization_msgs::Marker points;

    initialiseMarkerParams(points, color);

    // Set the points
    points.points = obstacles;

    return points;
}

visualization_msgs::Marker RvizUtils::displayPoint(geometry_msgs::Point obstacle, char color) {
    visualization_msgs::Marker point;

    initialiseMarkerParams(point, color);

    // Set the points
    point.points.push_back(obstacle);

    return point;
}

void RvizUtils::initialiseMarkerParams(visualization_msgs::Marker &marker, char color) {
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "debug";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;

    marker.type = visualization_msgs::Marker::POINTS;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;

    // Set the color
    switch (color) {
        case 'r':
            marker.color.r = 1.0f;
            break;
        case 'g':
            marker.color.g = 1.0f;
            break;
        case 'b':
            marker.color.b = 1.0f;
            break;
        default:
            marker.color.r = 1.0f;
    }
    // Set the transparency
    marker.color.a = 1.0;
}