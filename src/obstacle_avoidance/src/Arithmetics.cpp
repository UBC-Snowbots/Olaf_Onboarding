/*
 * Created By: John Shin
 * Created On: November 2nd, 2019
 * Description: arithmetic functions mainly for point32
 *
 */ 

#include<Arithmetics.h>
#include<geometry_msgs/Point32.h>
#include<geometry_msgs/Vector3.h>
#include<math.h>


/**
 * calculates the center point of a cluster
 *
 * calculates the mean Point32 given a vector of points
 *
 * @param cluster the given vector 
 *
 * return         the mean point
 */
geometry_msgs::Point32 Arithmetics::mean(std::vector<geometry_msgs::Point32> cluster){
	geometry_msgs::Point32 center;
	int size = cluster.size();
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	for (geometry_msgs::Point32 point: cluster){
		x += point.x;
		y += point.y;
		z += point.z;
	}

	center.x = x / size;
	center.y = y / size;
	center.z = z / size;
	return center;
}

/**
 * calculates the distance between 2 points
 *
 * calculates the distance between 2 geometry_msgs::Point32 points
 *
 * @param point1 one of the points to be compared
 * @param point2 the second point to be compared
 *
 * return        the distance between point1 and point2
 *
 */
float Arithmetics::distance(geometry_msgs::Point32 point1, geometry_msgs::Point32 point2){
	return sqrt(pow((point1.x-point2.x),2) + pow((point1.y-point2.y),2));
}

/**
 * compares the y coordinates of 2 points
 *
 * compares the y coordinates of 2 geometry_msgs::Point32 points
 *
 * @param point1 the first point to be compared
 * @param point2 the second point to be compared
 *
 * return        true if the first point is smaller than the second point in terms of the y value, returns false otherwise
 *
 */
bool Arithmetics::compareY(geometry_msgs::Point32 point1, geometry_msgs::Point32 point2){
	return point1.y < point2.y;
}

/**
 * calculates the middle point between 2 points
 *
 * calculates the middle point between 2 geometry_msgs::Point32 points
 *
 * @param point1 the first point
 * @param point2 the second point
 *
 * return        the midddle point between point1 and point2
 *
 */ 
geometry_msgs::Point32 Arithmetics::averagePoint(geometry_msgs::Point32 point1, geometry_msgs::Point32 point2){
	geometry_msgs::Point32 middle;
	middle.x = (point1.x + point2.x)/2;
	middle.y = (point1.y + point2.y)/2;
	middle.z = (point1.z + point2.z)/2;
	return middle;
}

/**
 * calculates the linear velocity 
 *
 * calculates the lienar velocity between origin and the destination point. 
 *     Assumes that the destination is located in the positive x-quadrant from the origin
 *
 * @param destination the destination point
 *
 * return             the linear velocity from origin to the destination point
 *
 */
geometry_msgs::Vector3 Arithmetics::linearVelocity(geometry_msgs::Point32 destination){
	geometry_msgs::Point32 origin;
	origin.x = 0;
	origin.y = 0;
	origin.z = 0;

	geometry_msgs::Vector3 linear;
	linear.x = distance(origin, destination);
	linear.y = 0;
	linear.z = 0;
	return linear;
}

/**
 * calculates the angular velocity
 *
 * calculates the angular velocity to the destination point from the origin
 * 	assumes that the destinatino is located in the positive x-quadrant from the origin
 *
 * @para destination the destination point
 *
 * return            the angular velocity from origin to the destination point
 *
 */
geometry_msgs::Vector3 Arithmetics::angularVelocity(geometry_msgs::Point32 destination){
	double angle = atan2(destination.y, destination.x);
	if(angle > M_PI){
		angle -= M_PI;
	}

	geometry_msgs::Vector3 angular;
	angular.x = 0;
	angular.y = 0;
	angular.z = angle;
	return angular;
}
