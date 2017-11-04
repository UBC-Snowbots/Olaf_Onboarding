//
// Created by min on 21/10/17.
//
#include <ObstacleAvoider.h>

float ObstacleAvoider::getAngularVel() {
    int rangeArraySize = getRangeArraySize();

    //TODO: make a tuple (dist, angle, x)?
    //TODO: save value of previous tuple (dist1, angle1, x1) for next iteration
    // for( int i = 0; i < rangeArraySize-1; i++ ) {
    int i = 0;
    while( i < rangeArraySize ) {
        float dist1 = _ranges[i];

        // skip i if distance is not valid
        if( !isDistanceInValidRange(dist1) ) {
            i++;
            continue;
        }

        // get next valid point - get dist2 and j
        int j = i+1;
        float dist2;
        while( j < rangeArraySize ) {
            dist2 = _ranges[j];
            if( isDistanceInValidRange(dist2) ) break;
            j++;
        }

        //TODO: handle case when dist1 is not matched with dist2?

        float angle1 = getAngleFromIndex(i);
//        float x1 = getX(dist1, angle1);
        obstacle obstacle1;
        obstacle1.distance = dist1;
        obstacle1.angle = angle1;

        float angle2 = getAngleFromIndex(j);
//        float x2 = getX(dist2, angle2);
        obstacle obstacle2;
        obstacle2.distance = dist2;
        obstacle2.angle = angle2;

        // found opening!
        if( getDistBetweenObstacles(obstacle1,obstacle2) >= _olaf_width  ) {
            // return the middle angle between two angles
            return (angle1+angle2)/2;
        }

        i++;
    }
    return 0.0;
}

float ObstacleAvoider::getDistBetweenObstacles(obstacle o1, obstacle o2) {
    float dx = getXBetweenObstacles(o1, o2);
    float dy = getYBetweenObstacles(o1, o2);

    return sqrt(pow(dx,2)+pow(dy,2));
}

float ObstacleAvoider::getXBetweenObstacles(obstacle o1, obstacle o2) {
    float x1 = getX(o1.distance, o1.angle);
    float x2 = getX(o2.distance, o2.angle);

    return abs(x1-x2);
}

float ObstacleAvoider::getYBetweenObstacles(obstacle o1, obstacle o2) {
    float y1 = getY(o1.distance, o1.angle);
    float y2 = getY(o2.distance, o2.angle);

    return abs(y1-y2);
}

void ObstacleAvoider::update(struct angleInfo angle_info, struct rangeInfo range_info, std::vector<float> ranges) {
    _ranges = ranges;

    _range_info = range_info;
    _angle_info = angle_info;
}