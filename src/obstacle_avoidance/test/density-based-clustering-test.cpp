#include"DensityBasedClustering.h"
#include<gtest/gtest.h>
#include<geometry_msgs/Point32.h>

#include<iostream>

class Clusters : public ::testing::Test{
	protected:
		virtual void SetUp(){
			float x = 2.0;
			float z = 0.0;
			//populate the left cluster
			float left = 2.0;
			while(left > 2.5){
				geometry_msgs::Point32 left_point;
				left_point.x = x;
			        left_point.y = left;
				left_point.z = z;
				left += 0.02;	
				cluster1.push_back(left_point);
			}
			//populate the right cluster
			float right = -2.0;
			while(right < -2.5){
				geometry_msgs::Point32 right_point;
				right_point.x = x;
				right_point.y = right;
				right_point.z = z;
				right -= 0.02;
				cluster2.push_back(right_point);
			}

			//combine all clusters
			all_data_points.insert(all_data_points.end(), cluster1.begin(), cluster1.end());
			all_data_points.insert(all_data_points.end(), cluster2.begin(), cluster2.end());
		}
		
		std::vector<geometry_msgs::Point32> cluster1;
		std::vector<geometry_msgs::Point32> cluster2;
		std::vector<geometry_msgs::Point32> all_data_points;
};


TEST_F(Clusters, densityBasedClustering){
	std::vector<std::vector<geometry_msgs::Point32>> clusters = DensityBasedClustering::findClusters(all_data_points);
	ASSERT_EQ(2, clusters.size());
}

TEST_F(Clusters, densityBasedClusteringWithNoise){
	geometry_msgs::Point32 noise1;
	geometry_msgs::Point32 noise2;;
	noise1.x = 1.0;
	noise1.y = 2.0;
	noise1.z = 0.0;
	noise2.x = -5.0;
	noise2.y = 3.0;
	noise2.z = 0.0;
	all_data_points.push_back(noise1);
	all_data_points.push_back(noise2);
	std::vector<std::vector<geometry_msgs::Point32>> clusters = DensityBasedClustering::findClusters(all_data_points);
	ASSERT_EQ(2, clusters.size());
}

TEST_F(Clusters, densityBasedClustering3Clusters){
	std::vector<geometry_msgs::Point32> cluster;
	float x = 5.0;
	float y = 0.1;
	float z = 0.0;
	while (y > 0.2){
		geometry_msgs::Point32 point;
		point.x = x;
		point.y = y;
		point.z = z;
		cluster.push_back(point);
	}
	all_data_points.insert(all_data_points.end(), cluster.begin(), cluster.end());
	std::vector<std::vector<geometry_msgs::Point32>> clusters = DensityBasedClustering::findClusters(all_data_points);
	ASSERT_EQ(3, clusters.size());
}
