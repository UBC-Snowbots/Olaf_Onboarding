/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Takes in a laser scan, and manages
 * 
 */
#include <LidarObstacleManager.h>
#include <ros/ros.h>

using namespace geometry_msgs;

LidarObstacleManager::LidarObstacleManager(double max_scan_distance, double cone_grouping_tolerance) :
        max_scan_distance(max_scan_distance),
        cone_grouping_tolerance(cone_grouping_tolerance) { }

void LidarObstacleManager::parseLaserScan(sensor_msgs::LaserScan laser_scan) {
    obstacles = constructObstacles(laser_scan);
    merged_obstacles = mergeObstacles(obstacles);
    pair<Point, Point> closest_pair = LinearAlgebra().getClosestPair(merged_obstacles[0], merged_obstacles[1]);

    hole = LinearAlgebra().getMiddlePoint(closest_pair.first, closest_pair.second);
}

vector<Point> LidarObstacleManager::constructObstacles(sensor_msgs::LaserScan laser_scan) {
    vector<Point> obstacles;
    int i = 0;
    float range_max = laser_scan.range_max;
    if (range_max > max_scan_distance)
        range_max = (float) max_scan_distance;
    for (float angle = laser_scan.angle_min; angle < laser_scan.angle_max; angle += laser_scan.angle_increment) {
        if (validateObstacle(laser_scan.ranges[i], laser_scan.range_min, range_max)) {
            obstacles.push_back(polarToCartesian(laser_scan.ranges[i], angle));
        }
        i++;
    }
    return obstacles;
}

bool LidarObstacleManager::validateObstacle(float range, float range_min, float range_max) {
    bool in_range = range > range_min && range < range_max;
    return !isnan(range) && in_range;

}

// Sort the vectors based on size
bool sortMerges(vector<Point> a, vector<Point> b) {
    return a.size() > b.size();
}

vector<vector<Point>> LidarObstacleManager::mergeObstacles(vector<Point> obstacles) {
    vector<vector<Point>> merged_obstacles;
    for (int i = 0; i < obstacles.size(); i++) {
        bool found_match = findMatch(merged_obstacles, obstacles[i]);

        // If didn't find a match start a new merge list.
        if (!found_match) {
            vector<Point> new_merge;
            new_merge.push_back(obstacles[i]);
            merged_obstacles.push_back(new_merge);
        }
    }
    // First two vectors should now correspond to left wall and right wall
    sort(merged_obstacles.begin(), merged_obstacles.end(), sortMerges);

    return merged_obstacles;
}

bool LidarObstacleManager::findMatch(vector<vector<Point>> &merged_obstacles,
                                     Point point) {
    for (int j = 0; j < merged_obstacles.size(); j++) {
        for (int k = 0; k < merged_obstacles[j].size(); k++) {
            if (LinearAlgebra().distanceBetweenPoints(merged_obstacles[j][k], point) < cone_grouping_tolerance) {
                merged_obstacles[j].push_back(point);
                return true;
            }
        }
    }
    return false;
}

Point LidarObstacleManager::polarToCartesian(float range, float theta) {
    Point cartesian_point;

    cartesian_point.x = range * cos(theta);
    cartesian_point.y = range * sin(theta);

    return cartesian_point;
}

vector<Point> LidarObstacleManager::getObstacles() {
    return obstacles;
}

vector<vector<Point>> LidarObstacleManager::getMergedObstacles() {
    return merged_obstacles;
}

Point LidarObstacleManager::getHole() {
    return hole;
}