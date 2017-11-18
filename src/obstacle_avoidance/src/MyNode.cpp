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

    vel_msg = avoidObstacles(laser_message); //Process ranges and return velocity message
    republishVelocity(vel_msg); //Publish velocity
}


//Algorithm for processing laser scan message
geometry_msgs::Twist MyClass::avoidObstacles(sensor_msgs::LaserScan laser_msg) {

    geometry_msgs::Twist vel_msg; //Initialize velocity message

    //Creates a vector of points that represents locations of obstacles in 2d points
    //Only points within the laser scanner's valid ranges will be considered
    std::vector<Point> points = createPoints(laser_msg);

    //locationOfGap contains point that represents center of largest gap
    Point locationOfGap = largestGap(points);

    //Find the angle of the center gap
    float ang_gap = atan(locationOfGap.y/locationOfGap.x);

    vel_msg.linear.x = 1; //Keep forward velocity constant
    vel_msg.angular.z = ang_gap; //Turn depending on angle of gap

    return vel_msg;
}


//Publish a velocity message
void MyClass::republishVelocity(geometry_msgs::Twist vel_msg_to_publish) {
    velocity_publisher.publish(vel_msg_to_publish);
}

/**
 * Construct a vector of points. The 0th index starts from negative angles (right side of robot)
 * @param laser_msg
 * @return
 */
std::vector<Point> MyClass::createPoints(sensor_msgs::LaserScan laser_msg) {

    float max_dist = laser_msg.range_max;
    float min_dist = laser_msg.range_min;
    float max_ang = laser_msg.angle_max;
    float min_ang = laser_msg.angle_min;
    float ang_inc = laser_msg.angle_increment;

    int numIndices = (max_ang-min_ang)/ang_inc;

    std::vector<float> ranges = laser_msg.ranges;
    std::vector<Point> points; // Holds 2d points

    for (int i = 0; i < numIndices; i++){
        //If range is valid (within min and max range), store as a 2d point
       if (ranges[i] <= max_dist && ranges[i] >= min_dist) {

           float angle = min_ang + i * ang_inc;
           float x = ranges[i] * cos(angle);
           float y = ranges[i] * sin(angle);

           Point point;
           point.x = x;
           point.y = y;

           points.push_back(point);
       }
    }
    return points;
}

/**
 * Find the largest gap and return a new point that represents the center
 * @param laser_msg
 * @param points vector of points must have size >= 2
 * @return
 */
Point MyClass::largestGap(std::vector<Point> points){

    Point centerPoint;

    float longest_dist = 0;

    for (int i = 0; i < points.size()-1; i++){
        float current_dist = getDist(points[i],points[i+1]); //Distance between successive points
        if (current_dist > longest_dist){
            longest_dist = current_dist;
            centerPoint.x = (points[i].x + points[i+1].x)/2.0;
            centerPoint.y = (points[i].y + points[i+1].y)/2.0;
        }
    }

    return centerPoint;
}

float MyClass::getDist(Point p1, Point p2){
    float x1 = p1.x;
    float y1 = p1.y;
    float x2 = p2.x;
    float y2 = p2.y;

    return sqrt(pow((y2-y1),2) + pow((x2-x1),2));
}