#include <gtest/gtest.h>
#include <LinearAlgebra.h>
#include <HoleTrackerDecision.h>
#include <geometry_msgs/Point.h>

geometry_msgs::Point origin;

// TODO: Create decision tests

int main(int obstacleTests, char **argv) {
    origin.x = 0;
    origin.y = 0;
    testing::InitGoogleTest(&obstacleTests, argv);
    return RUN_ALL_TESTS();
}