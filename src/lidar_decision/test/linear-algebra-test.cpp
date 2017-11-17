#include <gtest/gtest.h>
#include <LinearAlgebra.h>
#include <geometry_msgs/Point.h>

geometry_msgs::Point origin;
TEST(EuclideanDistance, PositivePointToOrigin) {
    
    geometry_msgs::Point test_point;
    test_point.x = 3;
    test_point.y = 4;
    EXPECT_DOUBLE_EQ(5.0, LinearAlgebra().distanceBetweenPoints(origin, test_point));
}

TEST(EuclideanDistance, NegativePointToOrigin) {
    geometry_msgs::Point test_point;
    test_point.x = -2;
    test_point.y = -4;
    EXPECT_DOUBLE_EQ(sqrt(20.0), LinearAlgebra().distanceBetweenPoints(origin, test_point));
}

TEST(EuclideanDistance, NegXPosYPointToOrigin) {
    geometry_msgs::Point test_point;
    test_point.x = -1;
    test_point.y = 10;
    EXPECT_DOUBLE_EQ(sqrt(101.0), LinearAlgebra().distanceBetweenPoints(origin, test_point));
}

TEST(EuclideanDistance, PosXNegYPointToOrigin) {
    geometry_msgs::Point test_point;
    test_point.x = 12;
    test_point.y = -3;
    EXPECT_DOUBLE_EQ(sqrt(153.0), LinearAlgebra().distanceBetweenPoints(origin, test_point));
}

TEST(EuclideanDistance, OriginToOrigin) {
    EXPECT_DOUBLE_EQ(0, LinearAlgebra().distanceBetweenPoints(origin, origin));
}

TEST(EuclideanDistance, PointToPoint) {
    geometry_msgs::Point test_point1, test_point2;
    test_point1.x = 5;
    test_point1.y = -3;
    test_point2.x = 8;
    test_point2.y = 4;

    EXPECT_DOUBLE_EQ(sqrt(58.0), LinearAlgebra().distanceBetweenPoints(test_point1, test_point2));
}

TEST(AngleTest, QuadrantOne) {
    geometry_msgs::Point test_point;
    test_point.x = 1;
    test_point.y = 1;

    EXPECT_DOUBLE_EQ(M_PI/4.0, LinearAlgebra().getAngleToPoint(test_point));
}

TEST(AngleTest, QuadrantTwo) {
    geometry_msgs::Point test_point;
    test_point.x = -1;
    test_point.y = 1;

    EXPECT_DOUBLE_EQ(3.0*M_PI/4.0, LinearAlgebra().getAngleToPoint(test_point));
}

TEST(AngleTest, QuadrantThree) {
    geometry_msgs::Point test_point;
    test_point.x = -1;
    test_point.y = -1;

    EXPECT_DOUBLE_EQ(-3.0*M_PI/4.0, LinearAlgebra().getAngleToPoint(test_point));
}

TEST(AngleTest, QuadrantFour) {
    geometry_msgs::Point test_point;
    test_point.x = 1;
    test_point.y = -1;

    EXPECT_DOUBLE_EQ(-M_PI/4.0, LinearAlgebra().getAngleToPoint(test_point));
}

TEST(AngleTest, QuadrantOneLessThan45Degrees) {
    geometry_msgs::Point test_point;
    test_point.x = 1;
    test_point.y = 0.5;

    EXPECT_DOUBLE_EQ(atan(0.5/1), LinearAlgebra().getAngleToPoint(test_point));
}

TEST(AngleTest, QuadrantFourMoreThan45Degrees) {
    geometry_msgs::Point test_point;
    test_point.x = 0.5;
    test_point.y = 1.0;

    EXPECT_DOUBLE_EQ(atan(1.0/0.5), LinearAlgebra().getAngleToPoint(test_point));
}

TEST(MiddlePoint, PointToOrigin) {
    geometry_msgs::Point test_point, middle_point;
    test_point.x = 10;
    test_point.y = 0.8;

    middle_point = LinearAlgebra().getMiddlePoint(test_point, origin);
    EXPECT_DOUBLE_EQ(5, middle_point.x);
    EXPECT_DOUBLE_EQ(0.4, middle_point.y);
}

TEST(MiddlePoint, PointToPoint) {
    geometry_msgs::Point test_point1, test_point2, middle_point;
    test_point1.x = 10;
    test_point1.y = 0.8;

    test_point2.x = 35;
    test_point2.y = 0.84;

    middle_point = LinearAlgebra().getMiddlePoint(test_point1, test_point2);
    EXPECT_DOUBLE_EQ(22.5, middle_point.x);
    EXPECT_DOUBLE_EQ(0.82, middle_point.y);
}

TEST(MiddlePoint, OriginToOrigin) {
    geometry_msgs::Point middle_point = LinearAlgebra().getMiddlePoint(origin, origin);

    EXPECT_DOUBLE_EQ(0, middle_point.x);
    EXPECT_DOUBLE_EQ(0, middle_point.y);
}

// TODO: ClosestPair tests

int main(int linearAlgebraTests, char **argv) {
    origin.x = 0;
    origin.y = 0;
    testing::InitGoogleTest(&linearAlgebraTests, argv);
    return RUN_ALL_TESTS();
}