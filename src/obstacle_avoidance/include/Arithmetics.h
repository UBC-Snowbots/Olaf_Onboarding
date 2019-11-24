#ifndef ARITHMETICS_H
#define ARITHMETICS_H

#include<geometry_msgs/Point32.h>
#include<geometry_msgs/Vector3.h>

class Arithmetics{
	
	public:
		static geometry_msgs::Point32 mean(std::vector<geometry_msgs::Point32>);

		static float distance(geometry_msgs::Point32, geometry_msgs::Point32);

		static bool compareY(geometry_msgs::Point32, geometry_msgs::Point32);
		
		static geometry_msgs::Point32 averagePoint(geometry_msgs::Point32, geometry_msgs::Point32);
		
		static geometry_msgs::Vector3 linearVelocity(geometry_msgs::Point32);
		
		static geometry_msgs::Vector3 angularVelocity(geometry_msgs::Point32);

};


#endif
