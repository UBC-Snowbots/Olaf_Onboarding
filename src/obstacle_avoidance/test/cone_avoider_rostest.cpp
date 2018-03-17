/*
 * Created By: Valerian Ratu
 * Created On: January 29, 2017
 * Modified By: Marinah Zhao
 * Modified On: March 10, 2018
 * Description: Integration testing for ConeAvoider
 */


#include <ConeAvoider.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

/**
 * This is the helper class which will publish and subscribe messages which will test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the subscriber
 *      getter function - to provide a way for gtest to check for equality of the message received
 */
class ConeAvoiderTest : public testing::Test{
protected:
    virtual void SetUp(){
        test_publisher = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
        test_subscriber = nh_.subscribe("/cmd_vel", 1, &ConeAvoiderTest::callback, this); //cone_avoider/

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
        twist_output.linear.x = 10;

    }

    ros::NodeHandle nh_;
    geometry_msgs::Twist twist_output;
    float num;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;

public:

    void callback(geometry_msgs::Twist::Ptr twist){
        twist_output = *twist;
    }
};

TEST_F(ConeAvoiderTest, middleHoleTwistMsgAfterLidarData) {

    // publishes a lidar scan to the test node
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.seq = 0;
    ros::Time time(1507411486);
    scan_msg.header.stamp = time;
    scan_msg.header.frame_id = "laser";
    scan_msg.angle_min = -2.4;
    scan_msg.angle_max = 2.1;
    scan_msg.angle_increment = 0.006;
    scan_msg.range_min = 0.02;
    scan_msg.range_max = 5.9;
    const float ranges_arr[] = {2.11, NAN, 8.0, 0.01, 4.5, 1.99};
    std::vector<float> ranges( ranges_arr, ranges_arr + sizeof( ranges_arr ) / sizeof( ranges_arr[0] ) );
    scan_msg.ranges = ranges;

    test_publisher.publish(scan_msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_EQ(0, twist_output.linear.y);
    EXPECT_EQ(0, twist_output.linear.z);
    EXPECT_EQ(0, twist_output.angular.x);
    EXPECT_EQ(0, twist_output.angular.y);

    EXPECT_EQ(0.6, twist_output.linear.x);
    EXPECT_NEAR(-3.1083, twist_output.angular.z, 0.0001);

}

TEST_F(ConeAvoiderTest, leftHoleTwistMsgAfterLidarData){

    // publishes a lidar scan to the test node
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.seq = 0;
    ros::Time time(1507411486);
    scan_msg.header.stamp = time;
    scan_msg.header.frame_id = "laser";
    scan_msg.angle_min = -2.4;
    scan_msg.angle_max = 2.1;
    scan_msg.angle_increment = 0.006;
    scan_msg.range_min = 0.02;
    scan_msg.range_max = 5.9;
    float ranges_arr[6] = {NAN, 6, 0.01, 4.3, NAN, 2.0};
    std::vector<float> ranges (ranges_arr, ranges_arr + sizeof(ranges_arr) / sizeof(ranges_arr[0]) );
    scan_msg.ranges = ranges;


    test_publisher.publish(scan_msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_EQ(0, twist_output.linear.y);
    EXPECT_EQ(0, twist_output.linear.z);
    EXPECT_EQ(0, twist_output.angular.x);
    EXPECT_EQ(0, twist_output.angular.y);

    EXPECT_EQ(0.6, twist_output.linear.x);
    EXPECT_NEAR(-3.1083, twist_output.angular.z, 0.0001);
}

TEST_F(ConeAvoiderTest, rightHoleTwistMsgAfterLidarData) {

    // publishes a lidar scan to the test node
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.seq = 0;
    ros::Time time(1507411486);
    scan_msg.header.stamp = time;
    scan_msg.header.frame_id = "laser";
    scan_msg.angle_min = -2.4;
    scan_msg.angle_max = 2.1;
    scan_msg.angle_increment = 0.006;
    scan_msg.range_min = 0.02;
    scan_msg.range_max = 5.9;
    float ranges_arr[6] = {4.5, NAN, 2.11, NAN, 0.018, NAN};
    std::vector<float> ranges (ranges_arr, ranges_arr + sizeof(ranges_arr) / sizeof(ranges_arr[0]) );
    scan_msg.ranges = ranges;


    test_publisher.publish(scan_msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_EQ(0, twist_output.linear.y);
    EXPECT_EQ(0, twist_output.linear.z);
    EXPECT_EQ(0, twist_output.angular.x);
    EXPECT_EQ(0, twist_output.angular.y);

    EXPECT_EQ(0.6, twist_output.linear.x);
    EXPECT_NEAR(-3.0927, twist_output.angular.z, 0.0001);
}

TEST_F(ConeAvoiderTest, noHoleTwistMsgAfterLidarData){

    // publishes a lidar scan to the test node
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.seq = 0;
    ros::Time time(1507411486);
    scan_msg.header.stamp = time;
    scan_msg.header.frame_id = "laser";
    scan_msg.angle_min = -2.4;
    scan_msg.angle_max = 2.1;
    scan_msg.angle_increment = 0.006;
    scan_msg.range_min = 0.02;
    scan_msg.range_max = 5.9;
    float ranges_arr[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<float> ranges (ranges_arr, ranges_arr + sizeof(ranges_arr) / sizeof(ranges_arr[0]) );
    scan_msg.ranges = ranges;

    test_publisher.publish(scan_msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_EQ(0, twist_output.linear.y);
    EXPECT_EQ(0, twist_output.linear.z);
    EXPECT_EQ(0, twist_output.angular.x);
    EXPECT_EQ(0, twist_output.angular.y);

    EXPECT_EQ(0.1, twist_output.linear.x);
    EXPECT_EQ(0, twist_output.angular.z);
}

int main(int argc, char **argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "cone_avoider_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
