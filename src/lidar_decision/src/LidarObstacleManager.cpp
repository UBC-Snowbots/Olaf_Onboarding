/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Manages obstacles by taking in a set of data points.
 * 
 */
#include <LidarObstacleManager.h>

LidarObstacleManager::LidarObstacleManager(sensor_msgs::LaserScan laser_scan, double max_scan_distance,
                                           double cone_grouping_tolerance) :
        max_scan_distance(max_scan_distance),
        cone_grouping_tolerance(cone_grouping_tolerance) {

    points = constructPoints(laser_scan);
    merged_points = mergePoints(points);
    pair<geometry_msgs::Point, geometry_msgs::Point> closest_pair = LinearAlgebra().getClosestPair(merged_points[0],
                                                                                                   merged_points[1]);
    hole = LinearAlgebra().getMiddlePoint(closest_pair.first, closest_pair.second);
}

vector<geometry_msgs::Point> LidarObstacleManager::constructPoints(sensor_msgs::LaserScan laser_scan) {
    vector<geometry_msgs::Point> points;
    int i = 0;
    for (float angle = laser_scan.angle_min; angle < laser_scan.angle_max; angle += laser_scan.angle_increment) {
        if (validatePoint(laser_scan.ranges[i], laser_scan.range_min, laser_scan.range_max)) {
            points.push_back(polarToCartesian(laser_scan.ranges[i], angle));
        }
        i++;
    }
    return points;
}

bool LidarObstacleManager::validatePoint(float range, float range_min, float range_max) {
    bool in_range = range > range_min && range < range_max;
    return !isnan(range) && in_range;

}

bool sortMerges(vector<geometry_msgs::Point> a, vector<geometry_msgs::Point> b) {
    return a.size() > b.size();
}

vector<vector<geometry_msgs::Point>> LidarObstacleManager::mergePoints(vector<geometry_msgs::Point> points) {
    vector<vector<geometry_msgs::Point>> merged_points;
    for (int i = 0; i < points.size(); i++) {
        bool found_match = findMatch(merged_points, points[i]);

        // If didn't find a match start a new merge list.
        if (!found_match) {
            vector<geometry_msgs::Point> new_merge;
            new_merge.push_back(points[i]);
            merged_points.push_back(new_merge);
        }
    }
    // First two vectors should now correspond to left wall and right wall
    sort(merged_points.begin(), merged_points.end(), sortMerges);

    return merged_points;
}

bool LidarObstacleManager::findMatch(vector<vector<geometry_msgs::Point>> &merged_points, geometry_msgs::Point point) {
    for (int j = 0; j < merged_points.size(); j++) {
        for (int k = 0; k < merged_points[j].size(); k++) {
            if (LinearAlgebra().distanceBetweenPoints(merged_points[j][k], point) < cone_grouping_tolerance) {
                merged_points[j].push_back(point);
                return true;
            }
        }
    }
    return false;
}

geometry_msgs::Point LidarObstacleManager::polarToCartesian(float range, float theta) {
    geometry_msgs::Point cartesian_point;

    cartesian_point.x = range * cos(theta);
    cartesian_point.y = range * sin(theta);

    return cartesian_point;
}

vector<geometry_msgs::Point> LidarObstacleManager::getPoints() {
    return points;
}

vector<vector<geometry_msgs::Point>> LidarObstacleManager::getMergedPoints() {
    return merged_points;
}

geometry_msgs::Point LidarObstacleManager::getHole() {
    return hole;
}