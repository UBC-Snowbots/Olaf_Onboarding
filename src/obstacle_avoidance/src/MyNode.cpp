/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <MyNode.h>
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <cstdlib> // for exit function
#include <iostream>



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
    std::string topic_to_subscribe_to = "scan";  //my_node will subscribe to subscribe topic
    int refresh_rate = 10;
    my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &MyClass::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("publish_topic"); //my_node will publish on publish topic
    uint32_t queue_size = 1;
    my_publisher = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);


}

void MyClass::subscriberCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Received message");

    float range_max = msg->range_max;

    sensor_msgs::PointCloud cloud;
    cloud.points.resize(msg->ranges.size());
    int a = (cloud.points).size();
    float angle_min = msg->angle_min;
    float angle_incr = msg->angle_increment;

    float * ranges = (float*)malloc(a* sizeof(float));
    for(int k = 0; k<a; k++){
        ranges[k] = msg->ranges[k];
    }

    convertToPointCloud(ranges, &cloud, a, angle_min, angle_incr);


    geometry_msgs::Point32 output_points;
    float hole_angle = locateTheHole(&cloud, &output_points, a, angle_min, angle_incr);


    geometry_msgs::Twist twist;
    MoveTowardsHole(&twist, hole_angle);


    my_publisher.publish(twist);



}

void MyClass::convertToPointCloud(float * ranges, sensor_msgs::PointCloud* cloud, int size, float angle_min, float angle_incr) {
    for(int i = 0; i<size; i++){
        cloud->points[i].x = ranges[i]*cos(angle_min+i*angle_incr);
        cloud->points[i].y = ranges[i]*sin(angle_min+i*angle_incr);
        cloud->points[i].z = 0;
    }
}

float MyClass::locateTheHole(sensor_msgs::PointCloud* cloud, geometry_msgs::Point32* output_points, int size, float angle_min, float angle_incr){
    float cloud_array[5];
    float mean_max = 0;
    float hole_angle;

    for(int i = 2; i < size-3; i++) {

        cloud_array[0] = cloud->points[i - 2].x;
        cloud_array[1] = cloud->points[i - 1].x;
        cloud_array[2] = cloud->points[i].x;
        cloud_array[3] = cloud->points[i + 1].x;
        cloud_array[4] = cloud->points[i + 2].x;

        float mean = 0;
        for(int j = 0; j < 5; j++){
            mean += cloud_array[j];
        }
        mean /= 5;

        if(mean >= mean_max) {
            mean_max = mean;
            hole_angle = angle_min + angle_incr * i;
            output_points->x = cloud->points[i].x;
            output_points->y = cloud->points[i].y;
            output_points->z = 0;
        }
    }
    return hole_angle;
}

void MyClass::MoveTowardsHole(geometry_msgs::Twist* twist, float hole_angle){
    if(hole_angle > 0.05){
        twist->angular.x = 0.0;
        twist->angular.y = 0.0;
        twist->angular.z = 0.01;

        twist->linear.x = 0.0;
        twist->linear.y = 0.0;
        twist->linear.z = 0.0;
    }
    else if(hole_angle < -0.05){
        twist->angular.x = 0.0;
        twist->angular.y = 0.0;
        twist->angular.z = -0.01;

        twist->linear.x = 0.0;
        twist->linear.y = 0.0;
        twist->linear.z = 0.0;
    }
    else{
        twist->linear.x = 0.5;
        twist->linear.y = 0.0;
        twist->linear.z = 0.0;
    }
}