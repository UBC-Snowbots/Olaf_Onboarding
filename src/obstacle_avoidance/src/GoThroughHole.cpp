/*
 * Created By: Ihsan Olawale
 * Created On: February 16th, 2019
 * Description: Uses LaserScan data of a setup of cones to move Olaf the robot
 *              through the gap in the cones
 */

#include <GoThroughHole.h>
#include <laser_geometry/laser_geometry.h>
#include <cmath>


GoThroughHole::GoThroughHole(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "scan";
    int refresh_rate = 10;
    my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &GoThroughHole::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("move_olaf");
    uint32_t queue_size = 1;
    my_publisher = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
}

void GoThroughHole::subscriberCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {
	sensor_msgs::PointCloud cloud = laserToPointCloud(*msg);
	geometry_msgs::Point32 center = findHole(cloud);
	geometry_msgs::Twist command = moveToHole(center);
	my_publisher.publish(command);
}

std::vector<int> GoThroughHole::findGaps(sensor_msgs::LaserScan msg) {
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

double distance (geometry_msgs::Point32 point1, geometry_msgs::Point32 point2) {
    return std::sqrt(std::pow(point1.x-point2.x,2)+std::pow(point1.y-point2.y,2)+std::pow(point1.z-point2.z,2));
}

geometry_msgs::Point32 GoThroughHole::findHole (sensor_msgs::PointCloud cloud) {
    // Use K-means algorithm with K=2 to filter data points into two walls
    // Because geometry_msgs::Point32 does not come with methods to compare, each wall is split into its x and y components
    std::vector<double> wall1x, wall1y, wall2x, wall2y, pre_wall1x, pre_wall1y;
    // left most point of cloud
    geometry_msgs::Point32 centroid1 = cloud.points.back();
    // right most point of cloud
    geometry_msgs::Point32 centroid2 = cloud.points[0];
    pre_wall1x = {0}, pre_wall1y = {0};
    while (pre_wall1x != wall1x || pre_wall1y != wall1y) {
        pre_wall1x = std::move(wall1x);
        pre_wall1y = std::move(wall1y);
        // assign each point to a wall
        for (int i = 0; i < cloud.points.size(); i++) {
            if (distance(cloud.points[i],centroid1) < distance(cloud.points[i],centroid2)) {
                wall1x.push_back(cloud.points[i].x);
                wall1y.push_back(cloud.points[i].y);
            }else {
                wall2x.push_back(cloud.points[i].x);
                wall2y.push_back(cloud.points[i].y);
            }
        }
        // recompute the centroids
        centroid1.x = 0, centroid1.y = 0, centroid1.z = 0;
        centroid2 = centroid1;
        for (int j = 0; j < wall1x.size(); j++) {
            centroid1.x += wall1x[j] / wall1x.size(); 
            centroid1.y += wall1y[j] / wall1x.size();
        }
        for (int j = 0; j < wall2x.size(); j++) {
            centroid2.x += wall2x[j] / wall2x.size();
            centroid2.y += wall2y[j] / wall2x.size();
        }
    }
    geometry_msgs::Point32 center;
    // find right most point of wall1, left most point of wall2
    // average them
    center.x = (wall1x[0] + wall2x.back())/2;
    center.y = (wall1y[0] + wall2y.back())/2;
    center.z = 0;
    return center;
}

sensor_msgs::PointCloud GoThroughHole::laserToPointCloud (sensor_msgs::LaserScan msg) {
	sensor_msgs::PointCloud cloud;
    	laser_geometry::LaserProjection projector_;
    	projector_.projectLaser(msg, cloud); 
	return cloud;
}

geometry_msgs::Twist GoThroughHole::stopOlaf (void) {
	geometry_msgs::Twist command;
	command.linear.x = 0;
	command.linear.y = 0;
	command.linear.z = 0;
	command.angular.z = 0;
	command.angular.y = 0;
	command.angular.x = 0;
	return command;
}

geometry_msgs::Twist GoThroughHole::moveToHole (geometry_msgs::Point32 center) {
	// move along line connecting robot to center
	geometry_msgs::Twist command;
	command.linear.y = 0;
	command.linear.z = 0;
	command.angular.x = 0;
	command.angular.y = 0;

	command.linear.x = std::sqrt(std::pow(center.x,2)+std::pow(center.y,2));
	command.angular.z = std::atan2(center.y, center.x);

        if (std::sqrt(std::pow(center.x,2)+std::pow(center.y,2)) < 0.001) {
		command = stopOlaf(); 
	}
        return command;
}
