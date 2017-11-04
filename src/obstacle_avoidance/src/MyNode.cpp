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

    vel_msg = avoidObstacles(laser_message); //Process ranges and return velocity
    republishVelocity(vel_msg); //Publish velocity
}


//Algorithm for processing laser scan message
geometry_msgs::Twist MyClass::avoidObstacles(sensor_msgs::LaserScan laser_msg) {

    geometry_msgs::Twist vel_msg; //Initialize velocity message
    vel_msg.linear.x = 0.5; //Keep forward velocity constant

    //ranges[0] to ranges[numIndices] represents the distances between
    //the min and max angle scan
    int numIndices = (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment;

    //These represent the refined index range for our gap search (starting at the cone edges)
    int startIndex = 0;
    int endIndex = numIndices;

    //Find startIndex
    for (int i=0; i<=numIndices; i++) {
        //If distance is within 2 meters
        if (laser_msg.ranges[i] <= 2) {
            startIndex = i;
            break;
        }
    }
    //Find endIndex
    for (int i=numIndices; i>=0; i--) {
        if (laser_msg.ranges[i] <= 2) {
            endIndex = i;
            break;
        }
    }
    //If we can't find either edge, or if we're surrounded by edges, just go forward
    if (startIndex == 0 && endIndex == numIndices) {
        return vel_msg;
    }

    int largestGap = 0; //current largest gap measured in indices
    int currentGap = 0;
    int endIndexGap; //end index of largest gap

    //Find the largest gap in the edge range
    for (int i = startIndex; i <= endIndex; i++) {
        if (laser_msg.ranges[i] > 2) {
            currentGap ++;
            if (currentGap > largestGap){
                largestGap = currentGap;
                endIndexGap = i;
            }
        }
        else {
            currentGap = 0;
        }
    }

    //Angle of gap is the angle that points to the midpoint of the largest gap
    float angle_of_gap = laser_msg.angle_min + ((endIndexGap - largestGap/2) * laser_msg.angle_increment);

    //Turn robot towards the gap
    if (angle_of_gap > 0){
        vel_msg.angular.z = -1;
    }
    else {
        vel_msg.angular.z = 1;
    }
    return vel_msg;
}


//Publish a velocity message
void MyClass::republishVelocity(geometry_msgs::Twist vel_msg_to_publish) {
    velocity_publisher.publish(vel_msg_to_publish);
}