/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <GoThroughHole.h>
#include <laser_geometry/laser_geometry.h>
#include <cmath>

MyClass::MyClass(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Obtains character from the parameter server (or launch file), sets '!' as default
    /*
    std::string parameter_name = "my_node/character";
    std::string default_character = "!";
    SB_getParam(nh, parameter_name, suffix, default_character);
    */

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "scan";
    int refresh_rate = 10;
    my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &MyClass::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("publish_topic");
    uint32_t queue_size = 1;
    my_publisher = private_nh.advertise<sensor_msgs::PointCloud>(topic, queue_size);
}

void MyClass::subscriberCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Received message");
    /*
    std::string input_string = msg->data.c_str();
    std::string new_msg = addCharacterToString(input_string, suffix);
    republishMsg(new_msg);
    */
    /*
    ROS_INFO("\nThe ranges are: ");
    for (int i = 0; i < msg->ranges.size(); i++) {
        ROS_INFO("%f", msg->ranges[i]);
    }
    */
    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*msg, cloud); 
    my_publisher.publish(cloud);
    /*
    ROS_INFO("The gaps and their intensities:");
    std::vector<int> gaps = findGaps(*msg);
    for (int i = 0; i < gaps.size(); i++) {
        ROS_INFO("%i", gaps[i]);
    }
    */
}
/*
std::string MyClass::addCharacterToString(std::string input_string, std::string suffix) {
    return input_string.append(suffix);
}
void MyClass::republishMsg(std::string msg_to_publish) {
    std_msgs::String string_to_publish;
    string_to_publish.data = msg_to_publish;
    my_publisher.publish(string_to_publish);
    ROS_INFO("Published message");
}
*/
std::vector<int> MyClass::findGaps(sensor_msgs::LaserScan msg) {
    std::vector<int> gaps;
    int element = 0;
    bool gapExists = true;
    gaps.push_back(0);
    for (int i = 0; i < msg.ranges.size(); i++) {
        if (std::isnan(msg.ranges[i])) {
            if (!gapExists) {
                gaps.push_back(0);
                element++;
            }
            gapExists = true;
            gaps[element]++;
        } else {
            gapExists = false;
        }
    }
    return gaps;
}
