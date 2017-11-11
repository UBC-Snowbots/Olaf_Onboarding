#include <MyNode.h>
#include <gtest/gtest.h>

TEST(AvoidObstacles, WhatWeAreTesting){

    //Takes in laser msg, outputs velocity msg
    EXPECT_EQ(nullptr, MyClass::avoidObstacles(nullptr));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}