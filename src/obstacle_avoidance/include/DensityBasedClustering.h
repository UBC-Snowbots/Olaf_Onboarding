#ifndef DENSITYBASEDCLUSTERING_H
#define DENSITYBASEDCLUSTERING_H

#include<vector>
#include<geometry_msgs/Point32.h>

class DensityBasedClustering{

	public:
		static std::vector<std::vector<geometry_msgs::Point32>> findClusters(std::vector<geometry_msgs::Point32>);

		static std::vector<int> getNeighbours(std::vector<geometry_msgs::Point32>&, int);

		static void expandCluster(int, std::vector<geometry_msgs::Point32>&, std::vector<int>&, int, std::vector<bool>&, std::vector<int>&);

		static std::vector<std::vector<geometry_msgs::Point32>> divideIntoClusters(std::vector<int>&, std::vector<geometry_msgs::Point32>&);

};

#endif 
