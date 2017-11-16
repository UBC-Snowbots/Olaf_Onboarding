/*
 * Created By: Valerian Ratu
 * Created On: January 29, 2017
 * Description: Integration testing for MyNode
 */

#include <MyNode.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

/**
 * This is the helper class which will publish and subscribe messages which will
 * test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the
 * subscriber
 *      getter function - to provide a way for gtest to check for equality of
 * the message recieved
 */
class MyNodeTest : public testing::Test {
protected:
  virtual void SetUp() {
    // test_publisher = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // test_subscriber = nh_.subscribe("/scan", 10, &MyNodeTest::lidarCallback_,
    // this);
    test_publisher = nh_.advertise<sensor_msgs::LaserScan>("/scan", 10);
    test_subscriber =
        nh_.subscribe("/cmd_vel", 10, &MyNodeTest::velCallback_, this);

    // Let the publishers and subscribers set itself up timely
    ros::Rate loop_rate(1);
    loop_rate.sleep();
  }

  ros::NodeHandle nh_;
  geometry_msgs::Twist vel_msg;
  // sensor_msgs::LaserScan lidar_data;
  ros::Publisher test_publisher;
  ros::Subscriber test_subscriber;

public:
  void AvoidObstacleClass::velCallback_(
      const geometry_msgs::Twist::ConstPtr &vel_message) {
    vel_msg.linear.x = vel_message->linear.x;
    vel_msg.linear.y = vel_message->linear.y;
    vel_msg.linear.z = vel_message->linear.z;
    vel_msg.angular.x = vel_message->angular.x;
    vel_msg.angular.y = vel_message->angular.y;
    vel_msg.angular.z = vel_message->angular.z;
  }

  /*void AvoidObstacleClass::lidarCallback_(const
     sensor_msgs::LaserScan::ConstPtr & laser_scan_message) {
              //get the lidar data from the subscribed topic
              lidar_data.angle_min = laser_scan_message->angle_min;
              lidar_data.angle_max = laser_scan_message->angle_max;
              lidar_data.angle_increment = laser_scan_message->angle_increment;
              //std::cout << "lidar data " << lidar_data.angle_max << std::endl;
              //std::cout << "lidar scan " << laser_scan_message->angle_max <<
     std::endl;

              int lidar_array_size = sizeOfLidarArray(lidar_data);

              lidar_data.ranges.resize(lidar_array_size);

              for (int i = 0; i < lidar_array_size; i++) {
                      lidar_data.ranges[i] = laser_scan_message->ranges[i];
     //edit
              }
      }*/
};

TEST_F(MyNodeTest, Implementing Rostest) {

  // publishes "Hello" to the test node
  // std_msgs::String msg;
  // msg.data = "Hello";
  // test_publisher.publish(msg);
  sensor_msgs::LaserScan lidar_data;

  test_publisher.publish(vel_msg)

      // Wait for the message to get passed around
      ros::Rate loop_rate(1);
  loop_rate.sleep();

  // spinOnce allows ros to actually process your callbacks
  // for the curious:
  // http://answers.ros.org/question/11887/significance-of-rosspinonce/
  ros::spinOnce();

  EXPECT_EQ("", lidar);
}

int main(int argc, char **argv) {
  // !! Don't forget to initialize ROS, since this is a test within the ros
  // framework !!
  ros::init(argc, argv, "obstacle_avoidance");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}