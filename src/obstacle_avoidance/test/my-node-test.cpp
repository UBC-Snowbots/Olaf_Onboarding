/*
 * Created By: Chris Heathe
 * Created On: november 11th, 2016
 * Description: Tests for MyNode
 */
#include <ObstacleAvoidance.h>
#include <gtest/gtest.h>


TEST(MyNode, targetAngleofZerotoTwist){
    geometry_msgs::Twist twist;
    twist.linear.x = 1;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    geometry_msgs::Twist returned_twist = ObstacleAvoidanceNode::targetAngletoTwist(0);

    EXPECT_EQ(twist.linear.x, returned_twist.linear.x);
    EXPECT_EQ(twist.angular.z, returned_twist.angular.z);
}

TEST(MyNode, targetAngleoOnePointFivetoTwist){
    geometry_msgs::Twist twist;
    twist.linear.x = 0.25;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 3;
    geometry_msgs::Twist returned_twist = ObstacleAvoidanceNode::targetAngletoTwist(1);

    EXPECT_EQ(twist.linear.x, returned_twist.linear.x);
    EXPECT_EQ(twist.angular.z, returned_twist.angular.z);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}