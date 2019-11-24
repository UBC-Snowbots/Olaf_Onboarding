#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/LaserScan.h>


int main(){

	rosbag::Bag bag;
	bag.open("2017-10-7-14-24-15.bag");

	for(rosbag::MessageInstance const m: rosbag::View(bag)){
		sensor_msgs::LaserScan::ConstPtr i = m.instantiate<sensor_msgs::LaserScan>();
		if (s != NULL){
			std::cout << i->ranges[1] << std::endl;
		}		
	}

	bag.close();
}
