/*
 * Created By: Ihsan Olawale
 * Created On: February 29, 2019
 * Description: Integration testing for GoThroughHole
 */


#include <GoThroughHole.h>
#include <gtest/gtest.h>
#include <ros/ros.h>


/**
 * This is the helper class which will publish and subscribe messages which will test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the subscriber
 *      getter function - to provide a way for gtest to check for equality of the message recieved
 */
class GoThroughHoleTest : public testing::Test{
protected:
    virtual void SetUp(){
        test_publisher = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
        test_subscriber = nh_.subscribe("/go_through_hole/move_olaf", 1, &GoThroughHoleTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    geometry_msgs::Twist message_output;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;

public:

    void callback(const geometry_msgs::Twist::ConstPtr& msg){
        message_output = *msg;
    }
};

TEST_F(GoThroughHoleTest, exclamationMarkAppend){

    // publishes a LaserScan message to the test node

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    // Make sure the robot is receiving the right Twist command
    // EXPECT_EQ("Hello!", message_output);
}


int main(int argc, char **argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "my_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
