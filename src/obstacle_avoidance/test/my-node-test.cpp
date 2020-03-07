/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: Tests for MyNode
 */

#include <MyNode.h>
#include <gtest/gtest.h>

int my_argc;
char ** my_argv;

TEST(MyNode, convertPointCloudTest){


    float angle_min = -2.00;
    float angle_max = 2.00;
    float angle_increment= 1;

    float * range_mat = (float*)malloc(5*sizeof(float));
    range_mat[0] = 0.5;
    range_mat[1] = 0.75;
    range_mat[2] = 1.0;
    range_mat[3] = 2.0;
    range_mat[4] = 3.0;


    sensor_msgs::PointCloud cloud;
    cloud.points.resize(5);


    MyClass test_class(my_argc, my_argv, "name");

    test_class.convertToPointCloud(range_mat, &cloud, 5, angle_min, angle_increment);


    EXPECT_FLOAT_EQ(-0.20807341827, cloud.points[0].x);
    EXPECT_FLOAT_EQ(-0.45464871341, cloud.points[0].y);
    EXPECT_FLOAT_EQ(0.0, cloud.points[0].z);

    EXPECT_FLOAT_EQ(0.4052267294, cloud.points[1].x);
    EXPECT_FLOAT_EQ(-0.6311032386, cloud.points[1].y);
    EXPECT_FLOAT_EQ(0.0, cloud.points[1].z);

    EXPECT_FLOAT_EQ(1.0, cloud.points[2].x);
    EXPECT_FLOAT_EQ(0.0, cloud.points[2].y);
    EXPECT_FLOAT_EQ(0.0, cloud.points[2].z);

    EXPECT_FLOAT_EQ(1.08060461174, cloud.points[3].x);
    EXPECT_FLOAT_EQ(1.68294196962, cloud.points[3].y);
    EXPECT_FLOAT_EQ(0.0, cloud.points[3].z);

    EXPECT_FLOAT_EQ(-1.24844050964, cloud.points[4].x);
    EXPECT_FLOAT_EQ(2.72789228048, cloud.points[4].y);
    EXPECT_FLOAT_EQ(0.0, cloud.points[4].z);


}

TEST(MyNode, LocateHoleTest){
    sensor_msgs::PointCloud cloud;
    cloud.points.resize(100);

    for(int i = 0; i<100;i++){
        cloud.points[i].x = 5.0;
        cloud.points[i].y = 50.0 - 1.0*i;
    }

    //The points very far away will form a hole in the wall, the center of which will be at index no. 22
    cloud.points[20].x = 50.0;
    cloud.points[21].x = 50.0;
    cloud.points[22].x = 50.0;
    cloud.points[23].x = 50.0;
    cloud.points[24].x = 50.0;


    geometry_msgs::Point32 output_points;
    MyClass test_class(my_argc, my_argv, "name");

    float hole_angle = test_class.locateTheHole(&cloud, &output_points, 100, -45.0, 1.0);

    EXPECT_FLOAT_EQ(-45.0+1.0*22, hole_angle);

}


TEST(MyNode, MoveToHole){
    MyClass test_class(my_argc, my_argv, "name");



    geometry_msgs::Twist twist1;
    test_class.MoveTowardsHole(&twist1, -45.0);
    EXPECT_FLOAT_EQ(0.0, twist1.angular.x);
    EXPECT_FLOAT_EQ(0.0, twist1.angular.y);
    EXPECT_FLOAT_EQ(-0.01, twist1.angular.z);
    EXPECT_FLOAT_EQ(0.0, twist1.linear.x);
    EXPECT_FLOAT_EQ(0.0, twist1.linear.y);
    EXPECT_FLOAT_EQ(0.0, twist1.linear.z);



    geometry_msgs::Twist twist2;
    test_class.MoveTowardsHole(&twist2, 45.0);
    EXPECT_FLOAT_EQ(0.0, twist2.angular.x);
    EXPECT_FLOAT_EQ(0.0, twist2.angular.y);
    EXPECT_FLOAT_EQ(0.01, twist2.angular.z);
    EXPECT_FLOAT_EQ(0.0, twist2.linear.x);
    EXPECT_FLOAT_EQ(0.0, twist2.linear.y);
    EXPECT_FLOAT_EQ(0.0, twist2.linear.z);

    geometry_msgs::Twist twist3;
    test_class.MoveTowardsHole(&twist3, 0.01);
    EXPECT_FLOAT_EQ(0.0, twist3.angular.x);
    EXPECT_FLOAT_EQ(0.0, twist3.angular.y);
    EXPECT_FLOAT_EQ(0.0, twist3.angular.z);
    EXPECT_FLOAT_EQ(0.5, twist3.linear.x);
    EXPECT_FLOAT_EQ(0.0, twist3.linear.y);
    EXPECT_FLOAT_EQ(0.0, twist3.linear.z);




}




int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    my_argc = argc;
    my_argv = argv;
    return RUN_ALL_TESTS();
}