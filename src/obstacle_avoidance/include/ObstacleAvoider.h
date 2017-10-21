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

    ObstacleAvoider(){};
    float getAngularVel();

    void update(struct angleInfo angle_info, struct rangeInfo range_info, std::vector<float> ranges);

private:
    float _olaf_width = 0.5;    // in m
    std::vector<float> _ranges;


    struct angleInfo _angle_info;

    struct rangeInfo _range_info;

    int getRangeArraySize() {
        return abs(_angle_info.angle_max - _angle_info.angle_min) / _angle_info.angle_increment + 1;
    }

    bool isDistanceInValidRange(float distance) {
        return !(distance < _range_info.range_min || distance > _range_info.range_max);
    }

    float getAngleFromIndex(int index) {
        return _angle_info.angle_min + index * _angle_info.angle_increment;
    }

    float getX(float distance, float angle) {
        return distance * cos(angle);
    }
};

#endif //PROJECT_OBSTACLEAVOIDER_H
