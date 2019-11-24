/*
 * Created By: John Shin
 * Created On: November 2nd, 2019
 * Description: functions for density based clustering
 *
 */

#include"DensityBasedClustering.h"
#include"Arithmetics.h"

#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Point32.h>
#include<vector>

const float EPSILON = 0.3;
const int MIN_NEIGHBOURS = 10;

/**
 * performs density based clustering
 *
 * performs density based clustering on array of 3d points. 
 *
 * @param 3dpoints sensor_msgs::PointCloud that has array of 3d points
 *
 * return          vector of vectors holding 3d points, each vector represents one clustering
 *
 */
std::vector<std::vector<geometry_msgs::Point32>> DensityBasedClustering::findClusters(std::vector<geometry_msgs::Point32> points){
	int size = points.size();
	std::vector<bool> visited (size, false);
	std::vector<int> assignment (size, 0);
	int cluster = 0;

	for (int i = 0; i < size; i++){
		if(!visited[i]){
			std::vector<int> neighbours = getNeighbours(points, i);
			if (neighbours.size() < MIN_NEIGHBOURS){
				// this point is a noise
			} else {
				cluster += 1;
				expandCluster(i, points, neighbours, cluster, visited, assignment);

			}	
		}
	}

	std::vector<std::vector<geometry_msgs::Point32>> clustering = divideIntoClusters(assignment, points);
	return clustering;
}

/**
 * returns the neighbouring index given a point
 *
 * returns the neighbouring point indexes that are within some fixed distance; EPSILON is a hyperparameter
 *
 * @param points the entire data points
 * @param index  the main point being analyzed
 *
 * return        a vector of neighbouring indexes
 */
std::vector<int> DensityBasedClustering::getNeighbours(std::vector<geometry_msgs::Point32> & points, int index){
	std::vector<int> neighbours;

	// look for close points to the left
	int left = index-1;
	while((left >= 0) && (Arithmetics::distance(points[left],points[index]) < EPSILON)){
		neighbours.push_back(left);
		left -= 1;
	}

	// look for close points to the right
	int right = index+1;
	while((right < points.size()) && (Arithmetics::distance(points[right],points[index]) < EPSILON)){
		neighbours.push_back(right);
		right += 1;
	}
	std::sort(neighbours.begin(), neighbours.end());
	return neighbours;
}

/**
 * expands the cluster 
 *
 * recursively expands the cluster to include points only if  those points are determined to be core point. To be a core point, that point has to have more than MIN_NEIGHBOURS that are within some EPSILON distance
 *
 * @param current_index the index of the current point that is already included in the cluster
 * @param points        the entire data points
 * @param neighbours    the indexes of neighbouring pionts of the current_index
 * @param visited       vector of boolean describing if a point has already been visited
 * @param assignment    cluster assignment of vector that are already visited
 */
void DensityBasedClustering::expandCluster(int current_index, std::vector<geometry_msgs::Point32> & points, std::vector<int> & neighbours, int cluster, std::vector<bool> & visited, std::vector<int> & assignment){
	if (visited[current_index]){
		return;
	}
	assignment[current_index] = cluster;
	visited[current_index] = true;
	for (int neighbour : neighbours){
		std::vector<int> neighbours_of_neighbour = getNeighbours(points, neighbour);
		if (neighbours_of_neighbour.size() < MIN_NEIGHBOURS){
			// this point is a noise
		} else {
			expandCluster(neighbour, points, neighbours_of_neighbour, cluster, visited, assignment);
		}
	}
} 

/**
 * divides the entire data points into clusters 
 *
 * divides the entire data points into clusters depending on their assignment
 *
 * @param assignment cluster assignment
 * @param points     the entire dataset
 *
 * return cluster groupings
 */
std::vector<std::vector<geometry_msgs::Point32>> DensityBasedClustering::divideIntoClusters(std::vector<int> & assignment, std::vector<geometry_msgs::Point32> & points){
	 std::vector<std::vector<geometry_msgs::Point32>> clusters;
	 int max = *std::max_element(assignment.begin(), assignment.end());
	 for(int i = 0; i<max; i++){
	 	std::vector<geometry_msgs::Point32> cluster;
 		clusters.push_back(cluster);
	 }
	 for(int j = 0; j < assignment.size(); j++){
		if (assignment[j] != 0){
	 		clusters[assignment[j]-1].push_back(points[j]);
		}
	 }
	 return clusters;
}
