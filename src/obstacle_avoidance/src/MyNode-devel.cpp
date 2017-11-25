/*
 * Created By: Chris Heathe
 * Created On:  October 28, 2017
 * Description: A node that will theoretically interpret lidar data,
 *              and help Olaf navigate through a hole in a row of cones
 *              Stress on *theoretically*
 */

#include <MyNode.h>

MyClass::MyClass(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Obtains character from the parameter server (or launch file), sets '!' as default
    std::string parameter_name = "my_node/character";
    std::string default_character = "!";
    SB_getParam(nh, parameter_name, suffix, default_character);

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "/scan";
    int refresh_rate = 10;
    my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &MyClass::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("/cmd_vel");
    uint32_t queue_size = 1;
    my_publisher = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
}

void MyClass::subscriberCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector <float> how_far = msg-> ranges;
    auto scan = *msg;
    for (int i = 217; i < (how_far.size() - 170); ++i) {
        //if (how_far[i] < msg->range_min || how_far[i] > msg->range_max || how_far[i] == NAN){
        if (!(how_far[i] > msg->range_min && how_far[i] < msg->range_max)) {
            how_far[i] = 5;
        }

    }

    float estimatedRange = 0;
    float maxEstimatedRange = 0;
    int maxEstimatedRangeLeftPoint = 0;
    int maxEstimatedRangeRightPoint = 0;
    for (int i = 222; i < (how_far.size() - 174); i++) {
        estimatedRange = (how_far[(i-4)] + how_far[(i-3)] + how_far[(i-2)] + how_far[(i-1)] + how_far[(i)] + how_far[(i+1)] + how_far[(i+2)] + how_far[(i+3)] + how_far[(i+4)])/9;
                if(estimatedRange > maxEstimatedRange) {
                    maxEstimatedRange = estimatedRange;
                    maxEstimatedRangeRightPoint = i;
                    maxEstimatedRangeLeftPoint =i;
                }
                else {
                    if (estimatedRange == maxEstimatedRange) {
                        maxEstimatedRangeLeftPoint = i;
                    }
                }
    }

   int angleIncrements2Hole = 0;
   angleIncrements2Hole = (maxEstimatedRangeLeftPoint + maxEstimatedRangeRightPoint)/2;
   float targetAngle = 0;
   targetAngle = (msg->angle_min) + ((angleIncrements2Hole)*(msg->angle_increment));
   ROS_INFO("%f", targetAngle);

    geometry_msgs::Twist returned_twist = targetAngletoTwist(targetAngle);
    ROS_INFO("%f", returned_twist.angular.z);
    ROS_INFO("%f", returned_twist.linear.x);
    publishVel(returned_twist);

}

geometry_msgs::Twist MyClass::targetAngletoTwist(float target_angle){
   geometry_msgs:: Twist twist;
    int gain = 1;
    twist.angular.z = gain*target_angle;

    if (target_angle >= 0) {
        twist.linear.x = 0.5/(1 + gain*target_angle);
    }
    if (target_angle < 0) {
        twist.linear.x = 0.5/(1 - gain*target_angle);
    }

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    return twist;
}


void MyClass::publishVel(geometry_msgs::Twist vel_to_publish) {
    my_publisher.publish(vel_to_publish);
    ROS_INFO("Published message");
}