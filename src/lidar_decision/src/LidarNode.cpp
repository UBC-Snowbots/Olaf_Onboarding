//
// Created by robyncastro on 10/11/17.
//

#include <LidarNode.h>

LidarNode::LidarNode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    initDecisionParams(private_nh);

    initObstacleManagerParams(private_nh);
    initSubscribers(nh);
    initPublishers(private_nh);

    // Setup Controller
    lidar_decision = LidarDecision(angular_vel_cap, linear_vel_cap, angular_vel_multiplier, linear_vel_multiplier,
                                   theta_scaling_multiplier);
    // Setup Rviz Utilities
    rviz_utils = RvizUtils();

}

void LidarNode::laserScanCallBack(const sensor_msgs::LaserScan laser_scan) {
    // Setup Obstacle Manager
    LidarObstacleManager obstacle_manager = LidarObstacleManager(laser_scan);

    // Merge Points based on distance
    vector<vector<geometry_msgs::Point>> merged_points = obstacle_manager.getMergedPoints();

    // Find the hole.
    geometry_msgs::Point hole = obstacle_manager.getHole();

    // Publish RViz markers
    vector<geometry_msgs::Point> hole_vector;
    hole_vector.push_back(hole);
    cone1_debug_publisher.publish(rviz_utils.displayPoints(merged_points[0], 'b'));
    cone2_debug_publisher.publish(rviz_utils.displayPoints(merged_points[1], 'g'));
    hole_debug_publisher.publish(rviz_utils.displayPoint(hole, 'r'));

    geometry_msgs::Twist follow_hole = lidar_decision.determineDesiredMotion(merged_points, hole);
    publishTwist(follow_hole);
}

void LidarNode::publishTwist(geometry_msgs::Twist twist_msg) {
    twist_publisher.publish(twist_msg);
}

void LidarNode::initDecisionParams(ros::NodeHandle private_nh) {
    SB_getParam(private_nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(private_nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(private_nh, "angular_vel_multiplier", angular_vel_multiplier, 1.0);
    SB_getParam(private_nh, "linear_vel_multiplier", linear_vel_multiplier, 1.0);
    SB_getParam(private_nh, "theta_scaling_multiplier", theta_scaling_multiplier, 1.0);
}

void LidarNode::initObstacleManagerParams(ros::NodeHandle private_nh) {
    SB_getParam(private_nh, "cone_grouping_tolerance", cone_grouping_tolerance, 1.0);
    SB_getParam(private_nh, "max_scan_distance", max_scan_distance, 1.0);
}

void LidarNode::initSubscribers(ros::NodeHandle nh) {
    std::string topic_to_subscribe_to = "/scan";
    int refresh_rate = 10;
    laser_scan_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &LidarNode::laserScanCallBack, this);
}

void LidarNode::initPublishers(ros::NodeHandle private_nh) {
    uint32_t queue_size = 1;
    std::string topic = private_nh.resolveName("cmd_vel");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
    std::string cone1_debug_topic = private_nh.resolveName("debug/cone1");
    cone1_debug_publisher = private_nh.advertise<visualization_msgs::Marker>(cone1_debug_topic, queue_size);
    std::string cone2_debug_topic = private_nh.resolveName("debug/cone2");
    cone2_debug_publisher = private_nh.advertise<visualization_msgs::Marker>(cone2_debug_topic, queue_size);
    std::string hole_debug_topic = private_nh.resolveName("debug/hole");
    hole_debug_publisher = private_nh.advertise<visualization_msgs::Marker>(hole_debug_topic, queue_size);
}