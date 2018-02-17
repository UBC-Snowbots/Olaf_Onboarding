/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <MyNode.h>

OlafNode::OlafNode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Obtains character from the parameter server (or launch file), sets '!' as default
    std::string parameter_name = "my_node/character";
    std::string default_character = "!";
    SB_getParam(nh, parameter_name, suffix, default_character);

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "subscribe_topic";
    int queue_size = 1;
//    my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &OlafNode::subscriberCallBack, this);

    topic_to_subscribe_to = "scan";
    my_subscriber = nh.subscribe(topic_to_subscribe_to, queue_size, &OlafNode::lidarSubscriberCallBack, this);
    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("/cmd_vel");
    my_publisher = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
}

void OlafNode::subscriberCallBack(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received message");
    std::string input_string = msg->data.c_str();
    std::string new_msg = addCharacterToString(input_string, suffix);
    republishMsg(new_msg);
}

void OlafNode::lidarSubscriberCallBack(sensor_msgs::LaserScan scan_msg) {
    ROS_INFO("Received message from lidar");
    ros::Time time = scan_msg.header.stamp;
    float angle_min = scan_msg.angle_min;
    float angle_max = scan_msg.angle_max;
    float angle_increment = scan_msg.angle_increment;
    float scan_time = scan_msg.scan_time;
    float range_min = scan_msg.range_min;
    float range_max = scan_msg.range_max;
    std::vector<float> ranges = scan_msg.ranges;

    int max_hole_size = 0;        // size of hole in number of indexes (angle increments)
    int max_hole_start_index = 0;     // start index (angle) of the start of a hole
    int cur_hole_size = 0;
    int cur_hole_start_index = 0;
    float MIN_HOLE_SIZE = 0;            // minimum size of hole robot can fit through
    float SAME_OB_VARIANCE = 5;         // variance in distance of same obstacle

    // iterate through the vector
    // remove values out of range
    for(std::vector<float>::iterator it = ranges.begin(); it != ranges.end(); ++it) {
        /* std::cout << *it; ... */
        //only process values within range
        if (*it >= range_min || *it <= range_max) {
            cur_hole_size = 0; //1
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

    // get the angles of the biggest hole and move towards the biggest hole
    float hole_start_angle = angle_min + max_hole_start_index*angle_increment;
    if (hole_start_angle > angle_max) {
        hole_start_angle = angle_max;
    }
    float hole_start_distance = ranges.at(max_hole_start_index);

    float hole_end_angle = angle_min + (max_hole_start_index + max_hole_size)*angle_increment;
    if (hole_end_angle > angle_max) {
        hole_end_angle = angle_max;
    }
    float hole_end_distance = ranges.at(max_hole_start_index + max_hole_size-1);
    if (std::isnan(hole_end_distance)) {
        hole_end_distance = hole_start_distance;
    }
    ROS_INFO("%f %f %i %i %f", hole_start_angle*57.29, hole_end_angle*57.29, max_hole_start_index, max_hole_size, angle_increment*57.29);

//    ROS_INFO("%lf %lf %lf %lf", hole_start_angle*57.29, hole_end_angle*57.29, hole_start_distance, hole_end_distance);

    // if the hole is not big enough, return
    double distance = sqrt(pow((hole_start_distance*cos(hole_start_angle))-(hole_end_distance * cos(hole_end_angle)), 2.0) +
                 pow((hole_start_distance*sin(hole_start_angle))-(hole_end_distance * sin(hole_end_angle)

    // find the middle of that hole and moves towards the middle of the hole((hole_start_distance * cos(hole_start_angle))+ (hole_end_distance * cos(hole_end_angle)))/2
//    float hole_mid_x = ((hole_start_distance * cos(hole_start_angle))+ (hole_end_distance * cos(hole_end_angle)))/2;
//    float hole_mid_y = ((hole_start_distance * sin(hole_start_angle))+ (hole_end_distance * sin(hole_en), 2.0));d_angle)))/2;
//
    double mid_angle = (hole_start_angle + hole_end_angle)/2;
//    // find the angle
//    if (hole_mid_x < 0) {
//        mid_angle = M_PI + atan(hole_mid_y/hole_mid_x);
//    } else if (hole_mid_x > 0) {
//        if (hole_mid_y > 0) {
//            mid_angle = atan(hole_mid_y/hole_mid_x);
//        } else {
//            mid_angle = 2*M_PI + atan(hole_mid_y/hole_mid_x);
//        }
//    } else if (hole_mid_x == 0) {
//        if (hole_mid_y > 0) {
//            mid_angle = M_PI/2;
//        } else {
//            mid_angle = 3*M_PI/2;
//        }
//    }

   // mid_angle = hole_start_angle;
    if (mid_angle > M_PI) {
        mid_angle = mid_angle - 2*M_PI;
    }

//    std_msgs::String string_to_publish;
//    ROS_INFO("%lf", mid_angle);
//    string_to_publish.data = mid_angle;
//    my_publisher.publish(string_to_publish);

  //   move towards the hole!
    geometry_msgs::Twist twist;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 1.3*mid_angle;
    twist.linear.x = 0.6;
    my_publisher.publish(twist);
}

std::string OlafNode::addCharacterToString(std::string input_string, std::string suffix) {
    return input_string.append(suffix);
}

void OlafNode::republishMsg(std::string msg_to_publish) {
    std_msgs::String string_to_publish;
    string_to_publish.data = msg_to_publish;
    my_publisher.publish(string_to_publish);
    ROS_INFO("Published message");
}
