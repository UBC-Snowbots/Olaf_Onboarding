/*
 * Created By: Robyn Castro
 * Created On: July 16th, 2016
 * Description: Manages obstacles by taking in a set of data points.
 * 
 */
#include <LidarObstacleManager.h>

LidarObstacleManager::LidarObstacleManager(sensor_msgs::LaserScan laser_scan) {
    points = constructPoints(laser_scan);
    merged_points = mergePoints();

    // Hole corresponds to the middle point between two lines.
    pair<geometry_msgs::Point, geometry_msgs::Point> closest_pair = LinearAlgebra().getClosestPair(merged_points[0],
                                                                                                   merged_points[1]);
    hole = LinearAlgebra().getMiddlePoint(closest_pair.first, closest_pair.second);
}

vector<geometry_msgs::Point> LidarObstacleManager::constructPoints(sensor_msgs::LaserScan laser_scan) {
    vector<geometry_msgs::Point> points;
    double angle = laser_scan.angle_min;
    int i = 0;
    int count = 0;
    while (angle < laser_scan.angle_max) {
        geometry_msgs::Point new_point;

        // Check for valid points
        if (!isnan(laser_scan.ranges[i]) && laser_scan.ranges[i] > laser_scan.range_min &&
            laser_scan.ranges[i] < laser_scan.range_max) {
            // Convert polar coordinates to cartesian points.
            new_point.x = laser_scan.ranges[i] * cos(angle);
            new_point.y = laser_scan.ranges[i] * sin(angle);

            // Add the point to the list
            points.push_back(new_point);
            count++;
        }

        // Parse next angle.
        i++;
        angle += laser_scan.angle_increment;
    }
    return points;
}

bool sortMerges(vector<geometry_msgs::Point> a, vector<geometry_msgs::Point> b) {
    return a.size() > b.size();
}

vector<vector<geometry_msgs::Point>> LidarObstacleManager::mergePoints() {
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
    bool found_match = false;
    int j = 0;
    while (j < merged_points.size() && !found_match) {
        for (int k = 0; k < merged_points[j].size(); k++) {
            if (LinearAlgebra().distanceBetweenPoints(merged_points[j][k], point) < 0.2) {
                found_match = true;
                merged_points[j].push_back(point);
                break;
            }
        }
        j++;
    }
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