/*
 * Created By: Min Gyo Kim
 * Created On: November 25, 2017
 * Description: Tests for ObstacleAvoiderNode
 */

#include <gtest/gtest.h>
#include <ObstacleAvoider.h>

ObstacleAvoider avoider;
ObstacleAvoider::rangeInfo range_info;
ObstacleAvoider::angleInfo angle_info;


TEST(ObstacleAvoider, TestGetRangeArraySize){
    // dummy range info
    range_info.range_min = 5;
    range_info.range_max = 10;

    // this is what we're testing
    angle_info.angle_min = -100;
    angle_info.angle_max = 100;
    angle_info.angle_increment = 50;

    avoider.update(angle_info, range_info, std::vector<float>());

    int size = avoider.getRangeArraySize();
    EXPECT_EQ(5,size);
}

TEST(ObstacleAvoider, TestIsDistanceInValidRange){
    // this is what we're testing
    range_info.range_min = 5;
    range_info.range_max = 10;

    // dummy value
    angle_info.angle_min = -100;
    angle_info.angle_max = 100;
    angle_info.angle_increment = 50;

    avoider.update(angle_info, range_info, std::vector<float>());

    EXPECT_EQ(false,avoider.isDistanceInValidRange(11));
    EXPECT_EQ(false,avoider.isDistanceInValidRange(4));
    EXPECT_EQ(true,avoider.isDistanceInValidRange(5));
    EXPECT_EQ(true,avoider.isDistanceInValidRange(10));
    EXPECT_EQ(true,avoider.isDistanceInValidRange(7));
}

TEST(ObstacleAvoider, TestGetAngleFromIndex) {
    // dummy range info
    range_info.range_min = 5;
    range_info.range_max = 10;

    // this is what we're testing
    angle_info.angle_min = -120;
    angle_info.angle_max = 120;
    angle_info.angle_increment = 4;

    avoider.update(angle_info, range_info, std::vector<float>());

    EXPECT_EQ(-120,avoider.getAngleFromIndex(0));
    EXPECT_EQ(-116,avoider.getAngleFromIndex(1));
    EXPECT_EQ(-112,avoider.getAngleFromIndex(2));
    EXPECT_EQ(120,avoider.getAngleFromIndex(60));
    EXPECT_EQ(0,avoider.getAngleFromIndex(30));
}

TEST(ObstacleAvoider, TestGetAngularVelSimple1) {
    range_info.range_min = 5;
    range_info.range_max = 10;

    angle_info.angle_min = 0;
    angle_info.angle_max = 1;
    angle_info.angle_increment = 1;

    std::vector<float> ranges = {5,5};

    avoider.update(angle_info, range_info, ranges);

    // distance between 5*cos(0) and 5*cos(1) is 2.3m, which is greater than olaf's width, 0.5m.
    // 0.5 = (0+1)/2
    EXPECT_EQ(0.5,avoider.getAngularVel());
}

TEST(ObstacleAvoider, TestGetAngularVelSimple2) {
    range_info.range_min = 5;
    range_info.range_max = 10;

    angle_info.angle_min = 0;
    angle_info.angle_max = 2;
    angle_info.angle_increment = 1;

    // 1 = (0+2)/2
    std::vector<float> ranges = {5,-5,5};
    avoider.update(angle_info, range_info, ranges);
    EXPECT_FLOAT_EQ(1,avoider.getAngularVel());

    // 0.5 = (0+1)/2
    ranges = {5,5,-5};
    avoider.update(angle_info, range_info, ranges);
    EXPECT_FLOAT_EQ(0.5,avoider.getAngularVel());

    // 1.5 = (1+2)/2
    ranges = {-5,5,5};
    avoider.update(angle_info, range_info, ranges);
    EXPECT_FLOAT_EQ(1.5,avoider.getAngularVel());
}

TEST(ObstacleAvoider, TestGetAngularVelSelectOpening) {
    range_info.range_min = 5;
    range_info.range_max = 10;

    angle_info.angle_min = 0;
    angle_info.angle_max = 1;
    angle_info.angle_increment = 0.1;

    // distance between point at angle 0 and 0.1 is 5*cos(0)-5*cos(0.1) = 0.0249, smaller than olaf's width, 0.5m
    // distance between point at angle 0.1 and 1 is 5*cos(0.1)-5*cos(1) = 2.27m, greater than olaf's width.

    // 0.55 = (0.1+1)/2
    std::vector<float> ranges = {5,5,-1,-1,-1,-1,-1,-1,-1,-1,5};
    avoider.update(angle_info, range_info, ranges);
    EXPECT_FLOAT_EQ(0.55,avoider.getAngularVel());

    // 0.45 = (0+0.9)/2
    ranges = {5,-1,-1,-1,-1,-1,-1,-1,-1,5,5};
    avoider.update(angle_info, range_info, ranges);
    EXPECT_FLOAT_EQ(0.45,avoider.getAngularVel());

    // 0.45 = (0+0.9)/2
    // Testing with nan as invalid ranges
    ranges = {5,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,5,5};
    avoider.update(angle_info, range_info, ranges);
    EXPECT_FLOAT_EQ(0.45,avoider.getAngularVel());

    // 0.45 = (0+0.9)/2
    // Testing with inf as invalid ranges
    ranges = {5,FP_INFINITE,FP_INFINITE,FP_INFINITE,FP_INFINITE,FP_INFINITE,FP_INFINITE,FP_INFINITE,FP_INFINITE,5,5};
    avoider.update(angle_info, range_info, ranges);
    EXPECT_FLOAT_EQ(0.45,avoider.getAngularVel());

    // Testing choosing maximum angle
    ranges = {5,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,5,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,5};
    angle_info.angle_max = 2;
    avoider.update(angle_info, range_info, ranges);
    EXPECT_FLOAT_EQ(1.45,avoider.getAngularVel());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}