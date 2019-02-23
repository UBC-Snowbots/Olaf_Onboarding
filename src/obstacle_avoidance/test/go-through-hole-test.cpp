/*
 * Created By: Ihsan Olawale
 * Created On: February 16th, 2019
 * Description: Tests for GoThroughHole
 */

#include <GoThroughHole.h>
#include <gtest/gtest.h>
#include <cmath>

// Testing stopOlaf
TEST(GoThroughHole, stopOlaf){

    geometry_msgs::Twist expected_command;

    expected_command.linear.x = 0;
    expected_command.linear.y = 0;
    expected_command.linear.z = 0;
    expected_command.angular.x = 0;
    expected_command.angular.y = 0;
    expected_command.angular.z = 0;

    geometry_msgs::Twist command = GoThroughHole::stopOlaf();

    EXPECT_EQ(expected_command.linear.x, command.linear.x);
    EXPECT_EQ(expected_command.linear.y, command.linear.y);
    EXPECT_EQ(expected_command.linear.z, command.linear.z);
    EXPECT_EQ(expected_command.angular.x, command.angular.x);
    EXPECT_EQ(expected_command.angular.y, command.angular.y);
    EXPECT_EQ(expected_command.angular.z, command.angular.z);
}

// Testing moveToHole
TEST(moveToHole, left_turn){
    geometry_msgs::Point32 center;

    center.x = 1;
    center.y = 1;
    center.z = 0;

    geometry_msgs::Twist command = GoThroughHole::moveToHole(center);
    geometry_msgs::Twist expected_command;

    expected_command.linear.x = std::sqrt(2);
    expected_command.angular.z = M_PI / 4;

    expected_command.linear.y = 0;
    expected_command.linear.z = 0;
    expected_command.angular.x = 0;
    expected_command.angular.y = 0;

    double abs_error = 0.001;
    // verify that the angle is in the right range
    EXPECT_NEAR(command.angular.z,expected_command.angular.z,abs_error);
    // verify that the position is also in right range
    EXPECT_NEAR(command.linear.x,expected_command.linear.x,abs_error);
}
TEST(moveToHole, right_turn){
    geometry_msgs::Point32 center;

    center.x = 1;
    center.y = -1;
    center.z = 0;

    geometry_msgs::Twist command = GoThroughHole::moveToHole(center);
    geometry_msgs::Twist expected_command;

    expected_command.linear.x = std::sqrt(2);
    expected_command.angular.z = -M_PI / 4;

    expected_command.linear.y = 0;
    expected_command.linear.z = 0;
    expected_command.angular.x = 0;
    expected_command.angular.y = 0;

    double abs_error = 0.001;
    // verify that the angle is in the right range
    EXPECT_NEAR(command.angular.z,expected_command.angular.z,abs_error);
    // verify that the position is also in right range
    EXPECT_NEAR(command.linear.x,expected_command.linear.x,abs_error);

}
TEST(moveToHole, no_turn){
    geometry_msgs::Point32 center;

    center.x = 0;
    center.y = 0;
    center.z = 0;

    geometry_msgs::Twist command = GoThroughHole::moveToHole(center);
    geometry_msgs::Twist expected_command;

    expected_command.linear.x = 0;
    expected_command.angular.z = 0;

    expected_command.linear.y = 0;
    expected_command.linear.z = 0;
    expected_command.angular.x = 0;
    expected_command.angular.y = 0;

    double abs_error = 0.001;
    // verify that the angle is in the right range
    EXPECT_NEAR(command.angular.z,expected_command.angular.z,abs_error);
    // verify that the position is also in right range
    EXPECT_NEAR(command.linear.x,expected_command.linear.x,abs_error);

}
TEST(moveToHole, wide_right_turn){
    geometry_msgs::Point32 center;

    center.x = -1;
    center.y = -1;
    center.z = 0;

    geometry_msgs::Twist command = GoThroughHole::moveToHole(center);
    geometry_msgs::Twist expected_command;

    expected_command.linear.x = std::sqrt(2);
    expected_command.angular.z = -3 * M_PI / 4;

    expected_command.linear.y = 0;
    expected_command.linear.z = 0;
    expected_command.angular.x = 0;
    expected_command.angular.y = 0;

    double abs_error = 0.001;
    // verify that the angle is in the right range
    EXPECT_NEAR(command.angular.z,expected_command.angular.z,abs_error);
    // verify that the position is also in right range
    EXPECT_NEAR(command.linear.x,expected_command.linear.x,abs_error);

}
TEST(moveToHole, wide_left_turn){
    geometry_msgs::Point32 center;

    center.x = -1;
    center.y = 1;
    center.z = 0;

    geometry_msgs::Twist command = GoThroughHole::moveToHole(center);
    geometry_msgs::Twist expected_command;

    expected_command.linear.x = std::sqrt(2);
    expected_command.angular.z = 3 * M_PI / 4;

    expected_command.linear.y = 0;
    expected_command.linear.z = 0;
    expected_command.angular.x = 0;
    expected_command.angular.y = 0;

    double abs_error = 0.001;
    // verify that the angle is in the right range
    EXPECT_NEAR(command.angular.z,expected_command.angular.z,abs_error);
    // verify that the position is also in right range
    EXPECT_NEAR(command.linear.x,expected_command.linear.x,abs_error);
}

// Testing findHole
TEST(findHole, straight_ahead){
    // create the PointCloud with noise
    // pass it into findHole
    // verify that the center is close to where the center should be
    sensor_msgs::PointCloud cloud;
    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);
}
TEST(findHole, really_close){
    // create the PointCloud with noise
    sensor_msgs::PointCloud cloud;
    // pass it into findHole
    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);
}
TEST(findHole, angled_right){
    // create the PointCloud with noise
    sensor_msgs::PointCloud cloud;
    // pass it in
    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);
}
TEST(findHole, angled_left){
    sensor_msgs::PointCloud cloud;
    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}