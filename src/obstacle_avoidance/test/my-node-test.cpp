#include <MyNode.h>
#include <gtest/gtest.h>

TEST(MyNode, testDist){
    Point p1;
    p1.x = 0;
    p1.y = 0;

    Point p2;
    p2.x = 3;
    p2.y = 4;

    EXPECT_EQ(5.0, MyClass::getDist(p1,p2));
}

TEST(MyNode, testLargestGap1){
    Point exp_point;
    std::vector<Point> testPoints;

    Point p1;
    p1.x = 0;
    p1.y = 0;

    Point p2;
    p2.x = 2;
    p2.y = 0;

    Point p3;
    p3.x = 7;
    p3.y = 0;

    Point p4;
    p4.x = 9;
    p4.y = 0;

    testPoints.push_back(p1);
    testPoints.push_back(p2);
    testPoints.push_back(p3);
    testPoints.push_back(p4);

    exp_point = MyClass::largestGap(testPoints);

    EXPECT_EQ(4.5, exp_point.x);
    EXPECT_EQ(0, exp_point.y);
}

TEST(MyNode, testLargestGap2){
    Point exp_point;
    std::vector<Point> testPoints;

    Point p1;
    p1.x = 0;
    p1.y = 0;

    Point p2;
    p2.x = 2;
    p2.y = 3;

    Point p3;
    p3.x = 1;
    p3.y = 0;

    Point p4;
    p4.x = 4;
    p4.y = 6;

    testPoints.push_back(p1);
    testPoints.push_back(p2);
    testPoints.push_back(p3);
    testPoints.push_back(p4);

    exp_point = MyClass::largestGap(testPoints);

    EXPECT_EQ(2.5, exp_point.x);
    EXPECT_EQ(3, exp_point.y);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}