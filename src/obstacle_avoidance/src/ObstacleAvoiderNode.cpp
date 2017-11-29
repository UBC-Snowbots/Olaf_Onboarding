/*
 * Created By: Min Gyo Kim
 * Created On: November 25, 2017
 * Description: ObstacleAvoiderNode - Node that outputs twist message based on obstacles
 */

#include <ObstacleAvoiderNode.h>

ObstacleAvoiderNode::ObstacleAvoiderNode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // TODO: get parameters for width of robot, forward velocity, rate
     std::string forward_vel_param = "obstacle_avoider_node/forward_vel";
     float forward_vel_default = 0.01;
    SB_getParam(nh, forward_vel_param, _forward_vel, forward_vel_default);
    std::cout << "foward velocity: " << _forward_vel << std::endl;

    std::string width_param = "obstacle_avoider_node/width";
    float width_default = 0.3;
    float width;
    SB_getParam(nh, width_param, width, width_default);
    obstacleAvoider.setOlafWidth(width);
    std::cout << "olaf width: " << width << std::endl;

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "/scan";
    int refresh_rate = 10;
    my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &ObstacleAvoiderNode::laserScanCallBack, this);

    // Setup Publisher(s)
    std::string topic_to_publish_to = "/cmd_vel";
    uint32_t queue_size = 1;
    my_publisher = nh.advertise<geometry_msgs::Twist>(topic_to_publish_to, queue_size);
}

void ObstacleAvoiderNode::goThoughCones() {
    geometry_msgs::Twist msg;
    msg.linear.x = _forward_vel;

    obstacleAvoider.update(_angle_info, _range_info, _ranges);
    msg.angular.z = obstacleAvoider.getAngularVel();
    std::cout << "angular z is " << msg.angular.z << std::endl;
    my_publisher.publish(msg);
    ROS_INFO("Published message");
}


void ObstacleAvoiderNode::laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ROS_INFO("Received message");

    _ranges = scan->ranges;

    _angle_info.angle_min = scan->angle_min;
    _angle_info.angle_max = scan->angle_max;
    _angle_info.angle_increment = scan->angle_increment;

    _range_info.range_min = scan->range_min;
    _range_info.range_max = scan->range_max;

    goThoughCones();
}
