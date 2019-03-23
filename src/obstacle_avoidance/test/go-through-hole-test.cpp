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
sensor_msgs::PointCloud createData (void) {
    sensor_msgs::PointCloud cloud;
    for (int i = 0; i < 16; i++) {
        geometry_msgs::Point32 point;
        point.x = 0;
        point.y = 0;
        point.z = 0;
        cloud.points.push_back(point);
    }
    
    cloud.points[0].x = 3;
    cloud.points[0].y = -3;

    cloud.points[1].x = 3.2;
    cloud.points[1].y = -2.8;
    
    cloud.points[2].x = 3;
    cloud.points[2].y = -2;

    cloud.points[3].x = 6;
    cloud.points[3].y = -2.1;

    cloud.points[4].x = 3;
    cloud.points[4].y = -1;
    
    cloud.points[5].x = 6;
    cloud.points[5].y = -1.5;

    cloud.points[6].x = 4;
    cloud.points[6].y = -1;

    cloud.points[7].x = 6;
    cloud.points[7].y = -1;

    for (int i = 8; i < 16; i++) {
        cloud.points[i] = cloud.points[15-i];
        cloud.points[i].y = -cloud.points[i].y;
    }

    return cloud;
}
TEST(assignToWall, accuracy){
    sensor_msgs::PointCloud cloud = createData();
    std::vector<double> wall1x, wall1y, wall2y, wall2x;
    GoThroughHole::assignToWalls(cloud, wall1x, wall1y, wall2x, wall2y);

    std::vector<double> expected_wall1x, expected_wall1y, expected_wall2x, expected_wall2y;
    for (int i = 0; i < 16; i++) {
        if (i < 8) {
            expected_wall1x.push_back(cloud.points[i].x);
            expected_wall1y.push_back(cloud.points[i].y);
        } else {
            expected_wall2x.push_back(cloud.points[i].x);
            expected_wall2y.push_back(cloud.points[i].y);
        }
    }
    EXPECT_EQ(expected_wall1x, wall1x);
    EXPECT_EQ(expected_wall1y, wall1y);
    EXPECT_EQ(expected_wall2x, wall2x);
    EXPECT_EQ(expected_wall2y, wall2y);
}
TEST(findBoundaryPoint, right){
    sensor_msgs::PointCloud cloud = createData();
    std::vector<double> wallx, wally;
    for (int i = 0; i < 8; i++) {
        wallx.push_back(cloud.points[i].x);
        wally.push_back(cloud.points[i].y);
    }
    geometry_msgs::Point32 point = GoThroughHole::findBoundaryPoint(wallx, wally, false);
    EXPECT_EQ(wallx[0], point.x);
    EXPECT_EQ(wally[0], point.y);
}
TEST(findBoundaryPoint, left){
    sensor_msgs::PointCloud cloud = createData();
    std::vector<double> wallx, wally;
    for (int i = 0; i < 8; i++) {
        wallx.push_back(cloud.points[i].x);
        wally.push_back(cloud.points[i].y);
    }
    geometry_msgs::Point32 point = GoThroughHole::findBoundaryPoint(wallx, wally, true);
    EXPECT_EQ(wallx.back(), point.x);
    EXPECT_EQ(wally.back(), point.y);
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
    for (int i = 0; i < cloud.points.size(); i++) {
        cloud.points[i].x -= 2.75;
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
    double angle = 45 * M_PI / 180;
    for (int i = 0; i < cloud.points.size(); i++) {
        double xnew = cloud.points[i].x*std::cos(-angle) - cloud.points[i].y*std::sin(-angle);
        double ynew = cloud.points[i].x*std::sin(-angle) + cloud.points[i].y*std::cos(-angle);
        cloud.points[i].x = xnew;
        cloud.points[i].y = ynew;
    }
    // pass it in
    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);

    geometry_msgs::Point32 expected;
    expected.x = 3*std::cos(-angle);
    expected.y = 3*std::sin(-angle);
    double abs_error = 0.001;
    EXPECT_NEAR(center.x, expected.x, abs_error);
    EXPECT_NEAR(center.y, expected.y, abs_error);
}
TEST(findHole, angled_left){
    sensor_msgs::PointCloud cloud = createData();
    double angle = 45 * M_PI / 180;
    for (int i = 0; i < cloud.points.size(); i++) {
        double xnew = cloud.points[i].x*std::cos(angle) - cloud.points[i].y*std::sin(angle);
        double ynew = cloud.points[i].x*std::sin(angle) + cloud.points[i].y*std::cos(angle);
        cloud.points[i].x = xnew;
        cloud.points[i].y = ynew;
    }

    geometry_msgs::Point32 center = GoThroughHole::findHole(cloud);

    geometry_msgs::Point32 expected;
    expected.x = 3*std::cos(angle);
    expected.y = 3*std::sin(angle);

    double abs_error = 0.001;
    EXPECT_NEAR(center.y, expected.y, abs_error);
    EXPECT_NEAR(center.x, expected.x, abs_error);
}
TEST(findHole, offset){
    sensor_msgs::PointCloud cloud = createData();
    for (int i = 0; i < cloud.points.size(); i++) {
        cloud.points[i].y += 3;
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
