//
// Created by min on 21/10/17.
//

#ifndef PROJECT_OBSTACLEAVOIDER_H
#define PROJECT_OBSTACLEAVOIDER_H

#include <math.h>
#include <cstdlib>
#include <vector>

class ObstacleAvoider {
public:
    struct angleInfo {
        float angle_min;
        float angle_max;
        float angle_increment;
    };

    struct rangeInfo {
        float range_min;
        float range_max;
    };

    struct obstacle {
        float distance;
        float angle;
    };

    ObstacleAvoider(){};
    float getAngularVel();

    void update(struct angleInfo angle_info, struct rangeInfo range_info, std::vector<float> ranges);
    int getRangeArraySize() {
        return abs(_angle_info.angle_max - _angle_info.angle_min) / _angle_info.angle_increment + 1;
    }

    bool isDistanceInValidRange(float distance) {
        return !(distance < _range_info.range_min || distance > _range_info.range_max) || (distance != distance);
    }

    float getAngleFromIndex(int index) {
        return _angle_info.angle_min + index * _angle_info.angle_increment;
    }

    void setOlafWidth(float width) {
        _olaf_width = width;
    }

    static float getX(float distance, float angle) {
        return distance * cos(angle);
    }

    static float getY(float distance, float angle) {
        return distance * sin(angle);
    }

    static float getDistBetweenObstacles(obstacle o1, obstacle o2);
    static float getXBetweenObstacles(obstacle o1, obstacle o2);
    static float getYBetweenObstacles(obstacle o1, obstacle o2);

private:
    float _olaf_width=0.5;    // in m
    std::vector<float> _ranges;

    struct angleInfo _angle_info;

    struct rangeInfo _range_info;

};

#endif //PROJECT_OBSTACLEAVOIDER_H
