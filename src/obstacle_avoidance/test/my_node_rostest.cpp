/*
 * Created By: Valerian Ratu
 * Created On: January 29, 2017
 * Description: Integration testing for MyNode
 */


//#include <MyNode.h>
#include <gtest/gtest.h>
#include <ObstacleAvoider.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


/**
 * This is the helper class which will publish and subscribe messages which will test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the subscriber
 *      getter function - to provide a way for gtest to check for equality of the message recieved
 */
class MyNodeTest : public testing::Test{
protected:
    virtual void SetUp(){
        test_publisher = nh_.advertise<sensor_msgs::LaserScan>("/scan", 1);
        test_subscriber = nh_.subscribe("/cmd_vel", 1, &MyNodeTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
//    std::string message_output;
    float message_output;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;

public:

    void callback(const geometry_msgs::Twist::ConstPtr& scan){
        message_output = scan->angular.z;
    }
};

TEST_F(MyNodeTest, getAngularVel){

    // publishes "Hello" to the test node
//    st    d_msgs::String msg;
//    msg.data = "Hello";
    sensor_msgs::LaserScan scan;

    scan.angle_min = 0;
    scan.angle_max = 1;
    scan.angle_increment = 0.1;

    scan.range_min = 5;
    scan.range_max = 10;

    scan.ranges = {5,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,FP_NAN,5,5};

    test_publisher.publish(scan);
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();


    // Wait for the message to get passed around
    loop_rate.sleep();



    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();

    EXPECT_FLOAT_EQ(0.45, message_output);
}


int main(int argc, char **argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "my_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}