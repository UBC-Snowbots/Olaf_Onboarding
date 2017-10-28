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

//Subscriber callback
void MyClass::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    //Take required information from received message
    laser_message.ranges = scan->ranges; //Range vector contains distances at different angles

    //The info below should stay constant so we should cache it somewhere after first run-through
    laser_message.angle_max = scan->angle_max;
    laser_message.angle_min = scan->angle_min;
    laser_message.angle_increment = scan->angle_increment;
    laser_message.range_max = scan->range_max;
    laser_message.range_min = scan->range_min;

    vel_msg = goIntoGap(laser_message); //Process ranges and return velocity
    republishVelocity(vel_msg); //Publish velocity
}

//Algorithm for processing laser scan message
geometry_msgs::Twist MyClass::goIntoGap(sensor_msgs::LaserScan laser_msg) {

    geometry_msgs::Twist vel_msg; //Initialize velocity message
    vel_msg.linear.x = 0.5; //Keep forward velocity constant

    //ranges[0] to ranges[numIndices] represents the distances between
    //the min and max angle scan are not necessarily equal distances from 0 (forward)
    //we should try tossing out extra unneeded data
    int numIndices = (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment;

    float angle_of_gap;

    //this iterates through all the ranges
    //we should find a lower and upper bound index for the "edge distance spikes"
    //then we can find the center angle of a gap within these bounds
    for (int i=0; i<=numIndices; i++) {
        float distance = laser_msg.ranges[i]; //distance in meters
        float angle = laser_msg.angle_min + (i * laser_msg.angle_increment); //In radians

        //if a distance is out of the max & min range we should toss it out

        if (distance == 3.2) {
            vel_msg.angular.z = 2; //Angular velocity
        }
    }

    //Turn robot towards the gap
    if (angle_of_gap>0){
        vel_msg.angular.z = -1;
    }
    else
        vel_msg.angular.z = 1;

    return vel_msg;
}

//Publish a velocity message
void MyClass::republishVelocity(geometry_msgs::Twist vel_msg_to_publish) {
    velocity_publisher.publish(vel_msg_to_publish);
}