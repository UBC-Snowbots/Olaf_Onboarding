#include <MyNode.h>

MyClass::MyClass(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Setup Subscriber to laser scan
    std::string topic_to_subscribe_to = "/scan";
    int refresh_rate = 10;
    laser_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &MyClass::processLaserScan, this);

    // Setup Publisher to twist
    std::string topic_to_publish_to = "/cmd_vel";
    int queue_size = 1;
    velocity_publisher = nh.advertise<geometry_msgs::Twist>(topic_to_publish_to, queue_size);
}

void MyClass::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    laser_message.ranges = scan->ranges;
}

void MyClass::solution() {

    if (laser_message.ranges[4] == 3.2)
        vel_msg.linear.x = 4;


    velocity_publisher.publish(vel_msg);
}