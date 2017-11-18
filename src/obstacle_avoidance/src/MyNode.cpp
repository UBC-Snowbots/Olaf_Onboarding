/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <MyNode.h>

MyClass::MyClass(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // TODO: get parameters for width of robot, forward velocity, rate
//     Obtains character from the parameter server (or launch file), sets '!' as default
     std::string parameter_name = "my_node/character";
     std::string default_character = "!";
    SB_getParam(nh, parameter_name, suffix, default_character);

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "/scan";
    int refresh_rate = 10;
    my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &MyClass::laserScanCallBack, this);

    // Setup Publisher(s)
    std::string topic_to_publish_to = "/cmd_vel";
    uint32_t queue_size = 1;
    my_publisher = nh.advertise<geometry_msgs::Twist>(topic_to_publish_to, queue_size);

//    goThoughCones();
}

void MyClass::goThoughCones() {
//  ros::Rate rate(_rate);

//  while(ros::ok()){
    geometry_msgs::Twist msg;
    msg.linear.x = _forward_vel;

    obstacleAvoider.update(_angle_info, _range_info, _ranges);
    msg.angular.z = obstacleAvoider.getAngularVel();
    std::cout << "angular z is " << msg.angular.z << std::endl;
    my_publisher.publish(msg);
    ROS_INFO("Published message");

//      rate.sleep();
//      ros::spinOnce();      //Notice this
//  }
}


void MyClass::laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ROS_INFO("Received message");

    _ranges = scan->ranges;

    _angle_info.angle_min = scan->angle_min;
    _angle_info.angle_max = scan->angle_max;
    _angle_info.angle_increment = scan->angle_increment;

    _range_info.range_min = scan->range_min;
    _range_info.range_max = scan->range_max;

    goThoughCones();
}
