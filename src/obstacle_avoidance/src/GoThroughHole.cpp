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
	command = moveToHole(center);
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

geometry_msgs::Point32 GoThroughHole::findHole (sensor_msgs::PointCloud cloud) {
	std::vector<geometry_msgs::Point32> boundaryPoints;
	boundaryPoints.push_back(cloud.points[0]);
	boundaryPoints.push_back(cloud.points[1]);
	boundaryPoints.push_back(cloud.points[cloud.points.size()-2]);
	boundaryPoints.push_back(cloud.points[cloud.points.size()-1]);
	std::vector<double> slopes(2);
	slopes[0] = (boundaryPoints[1].y-boundaryPoints[0].y)/(boundaryPoints[1].x-boundaryPoints[0].x);
	slopes[1] = (boundaryPoints[3].y-boundaryPoints[2].y)/(boundaryPoints[3].x-boundaryPoints[2].x);
	// iterate over points
	double slope;
	for (int i = 1; i < cloud.points.size(); i++) {
		// calculate the two max slope differences
		slope = (cloud.points[i].y-cloud.points[i-1].y)/(cloud.points[i].x-cloud.points[i-1].x);
		if (slope > slopes[0]) {
			slopes[0] = slope;
			boundaryPoints[1] = cloud.points[i];
			boundaryPoints[0] = cloud.points[i-1];
		}else {
			if (slope > slopes[1]) {
				slopes[1] = slope;
				boundaryPoints[3] = cloud.points[i];
				boundaryPoints[2] = cloud.points[i-1];
			}
		}
	}
	geometry_msgs::Point32 center;
	center.x = (boundaryPoints[1].x+boundaryPoints[2].x)/2;
	center.y = (boundaryPoints[1].y+boundaryPoints[2].y)/2;
	center.z = (boundaryPoints[1].z+boundaryPoints[2].z)/2;
	return center;
	// average the two and find the center
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
