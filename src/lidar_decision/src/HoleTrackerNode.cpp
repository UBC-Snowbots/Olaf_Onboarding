/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Takes in a laser scan and publishes a twist, and visualisation messages.
 *
 */
#include <HoleTrackerNode.h>

HoleTrackerNode::HoleTrackerNode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    initDecisionParams(private_nh);
    initObstacleManagerParams(private_nh);
    initSubscribers(nh);
    initPublishers(nh);

    lidar_decision = LidarDecision(angular_vel_cap, linear_vel_cap, angular_vel_multiplier, linear_vel_multiplier,
                                   theta_scaling_multiplier, max_distance_from_goal);

}

void HoleTrackerNode::laserScanCallBack(const sensor_msgs::LaserScan laser_scan) {
    // Setup Obstacle Manager
    LidarObstacleManager obstacle_manager = LidarObstacleManager(laser_scan, max_scan_distance, cone_grouping_tolerance);

    vector<vector<geometry_msgs::Point>> merged_points = obstacle_manager.getMergedPoints();

    // Find the hole.cd
    geometry_msgs::Point hole = obstacle_manager.getHole();

    // Publish RViz markers
    visualization_msgs::Marker::_color_type red = RvizUtils().createMarkerColor(1.0f, 0, 0, 1.0f);
    visualization_msgs::Marker::_color_type green = RvizUtils().createMarkerColor(0, 1.0f, 0, 1.0f);
    visualization_msgs::Marker::_color_type blue = RvizUtils().createMarkerColor(0, 0, 1.0f, 1.0f);
    visualization_msgs::Marker::_scale_type scale = RvizUtils().createrMarkerScale(0.1, 0.1, 0.1);

    string frame_id = "laser";
    string ns = "debug";

    cone1_debug_publisher.publish(RvizUtils().displayPoints(merged_points[0], blue, scale, frame_id, ns));
    cone2_debug_publisher.publish(RvizUtils().displayPoints(merged_points[1], green, scale, frame_id, ns));
    hole_debug_publisher.publish(RvizUtils().displayPoint(hole, red, scale, frame_id, ns));

    geometry_msgs::Twist follow_hole = lidar_decision.determineDesiredMotion(merged_points, hole);
    publishTwist(follow_hole);
}

void HoleTrackerNode::publishTwist(geometry_msgs::Twist twist_msg) {
    twist_publisher.publish(twist_msg);
}

void HoleTrackerNode::initDecisionParams(ros::NodeHandle private_nh) {
    SB_getParam(private_nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(private_nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(private_nh, "angular_vel_multiplier", angular_vel_multiplier, 1.0);
    SB_getParam(private_nh, "linear_vel_multiplier", linear_vel_multiplier, 1.0);
    SB_getParam(private_nh, "theta_scaling_multiplier", theta_scaling_multiplier, 1.0);
    SB_getParam(private_nh, "max_distance_from_goal", max_distance_from_goal, 1.0);
}

void HoleTrackerNode::initObstacleManagerParams(ros::NodeHandle private_nh) {
    SB_getParam(private_nh, "cone_grouping_tolerance", cone_grouping_tolerance, 0.2);
    SB_getParam(private_nh, "max_scan_distance", max_scan_distance, 1.0);
}

void HoleTrackerNode::initSubscribers(ros::NodeHandle nh) {
    std::string topic_to_subscribe_to = "/scan";
    uint32_t refresh_rate = 10;
    laser_scan_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &HoleTrackerNode::laserScanCallBack, this);
}

void HoleTrackerNode::initPublishers(ros::NodeHandle private_nh) {
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