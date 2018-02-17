/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Modified By: Marinah Zhao
 * Modified On: Feb 12, 2018
 * Description: A node that subscribes to a lidar topic publishing twist messages for the robot to move.
 */

#include <OlafNode.h>

OlafClass::OlafClass(int argc, char **argv, std::string node_name) {

    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "scan";
    int queue_size = 1;
    my_subscriber = nh.subscribe(topic_to_subscribe_to, queue_size, &OlafClass::lidarSubscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("/cmd_vel");
    my_publisher = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
}

void OlafClass::lidarSubscriberCallBack(sensor_msgs::LaserScan scan_msg) {

    ROS_INFO("Received message from lidar");
    ros::Time time = scan_msg.header.stamp;
    float angle_min = scan_msg.angle_min;
    float angle_max = scan_msg.angle_max;
    float angle_increment = scan_msg.angle_increment;
    float range_min = scan_msg.range_min;
    float range_max = scan_msg.range_max;
    std::vector<float> ranges = scan_msg.ranges;

    int max_hole_size = 0;            // size of hole in number of indexes (number of angle increments)
    int max_hole_start_index = 0;     // start index (angle) of the start of a hole
    int cur_hole_size = 0;
    int cur_hole_start_index = 0;

    // Iterate through the vector
    // Remove values out of range
    for(std::vector<float>::iterator it = ranges.begin(); it != ranges.end(); ++it) {

        //only process values within range
        if (*it >= range_min || *it <= range_max) {
            cur_hole_size = 0;
            cur_hole_start_index = it - ranges.begin();
        } else {
            //values outside of range are considered holes
            cur_hole_size++;

            // find the biggest hole in the vector
            // can add a threshold to the biggest hole
            if (cur_hole_size > max_hole_size) {
                max_hole_start_index = cur_hole_start_index;
                max_hole_size = cur_hole_size;
            }
        }
    }

    // Get the angles of the biggest hole
    float hole_start_angle = angle_min + max_hole_start_index*angle_increment;
    if (hole_start_angle > angle_max) {
        hole_start_angle = angle_max;
    }

    float hole_end_angle = angle_min + (max_hole_start_index + max_hole_size)*angle_increment;
    if (hole_end_angle > angle_max) {
        hole_end_angle = angle_max;
    }

    // Get the middle angle of the biggest hole
    // Possible improvements: use distances to calculate the angle of the middle of the hole
    double mid_angle = (hole_start_angle + hole_end_angle)/2.0;

    if (mid_angle > M_PI) {
        mid_angle = mid_angle - 2.0*M_PI;
    }

    // Move towards the hole!
    geometry_msgs::Twist twist;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 1.3*mid_angle;
    twist.linear.x = 0.6;
    my_publisher.publish(twist);
}
