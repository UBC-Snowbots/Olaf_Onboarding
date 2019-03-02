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

    EXPECT_EQ(command.linear.x,expected_command.linear.x);
    EXPECT_EQ(command.linear.y,expected_command.linear.y);
    EXPECT_EQ(command.linear.z, expected_command.linear.z);
    EXPECT_EQ(command.angular.x,expected_command.angular.x);
    EXPECT_EQ(command.angular.y,expected_command.angular.y);
    EXPECT_EQ(command.angular.z,expected_command.angular.z);
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
geometry_msgs::PointCloud createData (void) {
    geometry_msgs::PointCloud cloud;
    for (int i = 0; i < 16; i++) {
        geometry_msgs::Point32 point;
        point.x = 0;
        point.y = 0;
        point.z = 0;
        cloud.data.push_back(point);
    }
    
    cloud.data[0].x = 3;
    cloud.data[0].y = -3;

    cloud.data[1].x = 3.2;
    cloud.data[1].y = -2.8;
    
    cloud.data[2].x = 3;
    cloud.data[2].y = -2;

    cloud.data[3].x = 6;
    cloud.data[3].y = -2.1;

    cloud.data[4].x = 3;
    cloud.data[4].y = -1;
    
    cloud.data[5].x = 6;
    cloud.data[5].y = -1.5;

    cloud.data[6].x = 4;
    cloud.data[6].y = -1;

    cloud.data[7].x = 6;
    cloud.data[7].y = -1;

    for (int i = 8; i < 16; i++) {
        cloud.data[i] = cloud.data[15-i];
    }

    return cloud;
}
// Testing findHole
TEST(findHole, straight_ahead){
    // create the PointCloud with noise
    // pass it into findHole
    // verify that the center is close to where the center should be
    sensor_msgs::PointCloud cloud = createData();

    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);
    geometry_msgs::Point32 expected;
    expected.x = 3;
    expected.y = 0;

    double abs_error = 0.001;
    EXPECT_NEAR(center.y, expected.y, abs_error);
    EXPECT_GE(center.x, expected.x);
}
TEST(findHole, really_close){
    // create the PointCloud with noise
    sensor_msgs::PointCloud cloud = createData();
    for (int i = 0; i < cloud.data.size(); i++) {
        cloud.data[i].x -= 2.75;
    }
    // pass it into findHole
    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);
    geometry_msgs::Point32 expected;
    expected.x = 0.25;
    expected.y = 0;

    double abs_error = 0.001;
    EXPECT_NEAR(center.y, expected.y, abs_error);
    EXPECT_GE(center.x, expected.x);
}
TEST(findHole, angled_right){
    // create the PointCloud with noise
    sensor_msgs::PointCloud cloud = createData();
    for (int i = 0; i < cloud.data.size(); i++) {
        cloud.data[i].x = std::sqrt(1.0/2)*(cloud.data[i].x + cloud.data[i].y);
        cloud.data[i].y = std::sqrt(1.0/2)*(-cloud.data[i].x + cloud.data[i].y);
    }
    // pass it in
    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);

    geometry_msgs::Point32 expected;
    expected.x = std::sqrt(1.0/2)*(3+0);
    expected.y = std::sqrt(1.0/2)*(-3+0);
    double abs_error = 0.001;
    // EXPECT_NEAR(center.x, expected.x, abs_error);
    // EXPECT_GE(center.y, expected.y);
}
TEST(findHole, angled_left){
    sensor_msgs::PointCloud cloud = createData();
    for (int i = 0; i < cloud.data[i]; i++) {
        cloud.data[i].x = std::sqrt(1.0/2)*(cloud.data[i].x - cloud.data[i].y);
        cloud.data[i].y = std::sqrt(1.0/2)*(cloud.data[i].x + cloud.data[i].y);
    }

    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);

    geometry_msgs::Point32 expected;
    expected.x = std::sqrt(1.0/2)*(3  - 0);
    expected.y = std::sqrt(1.0/2)*(3 + 0);

    double abs_error = 0.001;
    // EXPECT_NEAR(center, expected, abs_error);
    // EXPECT_GE(center, expected);
}
TEST(findHole, offset){
    sensor_msgs::PointCloud cloud = createData();
    for (int i = 0; i < cloud.data.size(); i++) {
        cloud.data[i].y += 3;
    }
    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);

    geometry_msgs::Point32 expected;
    expected.x = 3;
    expected.y = 3;

    double abs_error = 0.001;
    EXPECT_NEAR(center.y, expected.y, abs_error);
    EXPECT_GE(center.x, expected.x);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
