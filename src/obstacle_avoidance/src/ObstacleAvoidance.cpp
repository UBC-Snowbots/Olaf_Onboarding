/*
 * Created By: John Shin
 * Created On: Oct 29, 2019
 * Description: A node that gives an instruction to Olaf to safely go through a hole
 */

#include<ObstacleAvoidance.h>
#include"Arithmetics.h"
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Point32.h>
#include<DensityBasedClustering.h>
#include<laser_geometry/laser_geometry.h>

ObstacleAvoidance::ObstacleAvoidance(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "/scan";
    int refresh_rate = 10;
    my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &ObstacleAvoidance::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("/twist_topic");
    uint32_t queue_size = 1;
    my_publisher = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
}


void ObstacleAvoidance::subscriberCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {
    ROS_INFO("received a scan");
    // convert laser scan to point cloud
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*laser_scan, cloud);
    std::vector<geometry_msgs::Point32> points(std::begin(cloud.points), std::end(cloud.points));
   
    //filter though the points
    std::vector<geometry_msgs::Point32> filtered_points;
    for (geometry_msgs::Point32 point : points){
    	if (std::isnan(point.x) || std::isnan(point.y)){
		// filter infinities
	} else {
		filtered_points.push_back(point);
	}
    }

    //perform density based clustering
    std::vector<std::vector<geometry_msgs::Point32>> clusters = DensityBasedClustering::findClusters(filtered_points);    

    // if there are more than 2 clsters, only keep the 2 clusters with the smallest |y| value. Others will be considered as noise
    std::vector<geometry_msgs::Point32> left_wall;
    std::vector<geometry_msgs::Point32> right_wall;
    // pick the 2 clusters centered with the smallest |y| value
    float y_smallest = FLT_MAX;
    float y_small = FLT_MAX;
    int index_smallest = 0;  
    int index_small = 0;    
    int i= 0;
    for (std::vector<geometry_msgs::Point32> cluster : clusters){
		geometry_msgs::Point32 center = Arithmetics::mean(cluster);
		if(abs(center.y) < abs(y_smallest)){
			y_small = y_smallest;
			index_small = index_smallest;

			y_smallest = center.y;
			index_smallest = i;
		} else if (abs(center.y) < abs(y_small)){
			y_small = center.y;
			index_small = i;
		}
		i += 1;
    }
   
    //determine the left and right wall
    int left, right;
    left = 0;
    right = 0;
    if (y_smallest < y_small){
    	left = index_small;
	right = index_smallest;
    } else {
    	left = index_smallest;
	right = index_small;
    }
    left_wall = clusters[left];
    right_wall = clusters[right];

    //identify the window
    std::sort(left_wall.begin(), left_wall.end(), Arithmetics::compareY);
    std::sort(right_wall.begin(), right_wall.end(), Arithmetics::compareY);

    geometry_msgs::Point32 window_left = left_wall.front();
    geometry_msgs::Point32 window_right = right_wall.back();
 
    // using window_left and window_right return the middle point
    geometry_msgs::Point32 window_middle = Arithmetics::averagePoint(window_left, window_right);

    // construct and publish a Twist msg
    geometry_msgs::Twist twist_msg;
    twist_msg.linear = Arithmetics::linearVelocity(window_middle);
    twist_msg.angular = Arithmetics::angularVelocity(window_middle);    
    republishMsg(twist_msg);
}

void ObstacleAvoidance::republishMsg(geometry_msgs::Twist twist_data) {
    my_publisher.publish(twist_data);
    ROS_INFO("twist message published");
}
