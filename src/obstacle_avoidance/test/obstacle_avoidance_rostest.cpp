/*
 * Created By: John Shin
 * Created On: November 23, 2019
 * Description: Integration testing for ObstacleAvoidance
 */


#include "ObstacleAvoidance.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include<ros/console.h>
#include<log4cxx/logger.h>

/**
 * This is the helper class which will publish and subscribe messages which will test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the subscriber
 *      getter function - to provide a way for gtest to check for equality of the message recieved
 */
class ObstacleAvoidanceTest : public testing::Test{
protected:
    virtual void SetUp(){
	std::string topic = private_nh.resolveName("/scan");
	test_publisher = nh_.advertise<sensor_msgs::LaserScan>(topic,1);
        test_subscriber = nh_.subscribe("/twist_topic", 1, &ObstacleAvoidanceTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle private_nh;
    ros::NodeHandle nh_;
    float lx, ly, lz, ax, ay, az;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;

public:

    void callback(const geometry_msgs::Twist::ConstPtr & twist_data){
        ROS_INFO("MESSAGE RECEIVED");
        lx = twist_data->linear.x;
	ly = twist_data->linear.y;
	lz = twist_data->linear.z;
	ax = twist_data->angular.x;
	ay = twist_data->angular.y;
	az = twist_data->angular.z;	
    }
};

TEST_F(ObstacleAvoidanceTest, goThroughHole){
    // publishes the first laser data from the given bag file
    sensor_msgs::LaserScan::ConstPtr scan_data; 	
    rosbag::Bag bag;
    bag.open("/home/john/Documents/snowbots/nav_challenge/Nav_onboarding/src/obstacle_avoidance/test/2017-10-07-14-24-15.bag");	

    std::vector<rosbag::MessageInstance> message_vector;

    for (rosbag::MessageInstance const m:rosbag::View(bag)){
	message_vector.push_back(m);
    }

    scan_data = message_vector[0].instantiate<sensor_msgs::LaserScan>();
    bag.close();

    test_publisher.publish(scan_data);
    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_NEAR(2.75, lx, 0.1);
    EXPECT_FLOAT_EQ(0.0, ly);
    EXPECT_FLOAT_EQ(0.0, lz);
    EXPECT_FLOAT_EQ(0.0, ax);
    EXPECT_FLOAT_EQ(0.0, ay);
    EXPECT_NEAR(0.0764, az, 0.1);
}


int main(int argc, char **argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "obstacle_avoidance_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
