#include <MyNode.h>
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
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;
    float lin_x_vel;
    float ang_z_vel;

public:

    void callback(const geometry_msgs::Twist::ConstPtr vel){
        lin_x_vel = vel->linear.x;
        ang_z_vel = vel->angular.z;
    }
};

TEST_F(MyNodeTest, testObstacleAvoidance){

    //Publishes a custom laser_message to the test node
    sensor_msgs::LaserScan laser_msg;

    laser_msg.angle_max = 2.0; //In Radians
    laser_msg.angle_min = -2.0;  //In Radians
    laser_msg.angle_increment = 0.5;
    laser_msg.range_max = 5.0; //5m
    laser_msg.range_min = 0.0; //0m
    laser_msg.ranges = {6.0, 1.5, 3.0, 2.0, 2.0, 6.0, 7.0, 6.0, 7.0}; //Should have 9 ranges
    //Note that ranges start at -angles to +angles

    test_publisher.publish(laser_msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_EQ(1, lin_x_vel);
    EXPECT_EQ(2, ang_z_vel); //?
}


int main(int argc, char **argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "my_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}