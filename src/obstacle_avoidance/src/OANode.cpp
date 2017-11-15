/*
 * Created By: Marcus Swift
 * Created On: October 20th, 2017
 * Description: A node that subscribes to the laser scan message and outputs
 * a twist message that allows the vehicle to avoid obstacles
 */

#include <OANode.h>

ObstacleAvoidance::ObstacleAvoidance() {
  // initialise velocity
  vel_msg.linear.x = linear_vel;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;
}

ObstacleAvoidance::ObstacleAvoidance(int argc, char **argv,
                                       std::string node_name) {
  // Setup NodeHandle
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  // Setup Subscriber/Publisher
  twist_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  laser_scan_subscriber =
      nh.subscribe("/scan", 10, &ObstacleAvoidance::lidarCallback, this);

  // initialise velocity
  vel_msg.linear.x = linear_vel;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;
  std::cout << "completed construction \n";
}

ObstacleAvoidance::~ObstacleAvoidance() {
  //Stop vehicle when node completes
  setLinearVelMsg(0);
  setAngVelMsg(0);
  publishVel();
  ros::spinOnce();
  std::cout << "completed destruction \n";
}

ObstacleAvoidance::avoidance ObstacleAvoidance::moveToAvoidCollision(float lidar_distance,
                                                   float lidar_angle) {
  ObstacleAvoidance::avoidance move_to_avoid_collision;
  
  //determine if the lidar point is in front of the vehicle
  if ((lidar_angle < M_PI / 2) && (lidar_angle > -M_PI / 2)) {
    float x = lidar_distance * sin(lidar_angle);
	//determine if the lidar point is directly in front of the vehicle
    if ((x < vehicle_width / 2) && (x > -vehicle_width / 2)) {
      if (x <= 0) {
        move_to_avoid_collision = left;
      } else {
        move_to_avoid_collision = right;
      }

    } else {
      move_to_avoid_collision = no_avoidance;
    }

  } else {
    move_to_avoid_collision = no_avoidance;
  }
  return move_to_avoid_collision;
}



double ObstacleAvoidance::angVelocityToAvoidCollision() {

  double critical_distance = min_critical_distance;
  double ang_velocity = 0;
  ObstacleAvoidance::avoidance move_to_avoid_obstacle;

  for (int i = 0; i < lidar_data.ranges.size(); i++) {
    if ((lidar_data.ranges[i] != NAN) && (lidar_data.ranges[i] > 0.03) &&
        (lidar_data.ranges[i] < 5)) {
      // check if lidar point is going to collide
      move_to_avoid_obstacle = moveToAvoidCollision(
          lidar_data.ranges[i],
          (lidar_data.angle_min + lidar_data.angle_increment * i));
      // if most immediate threat/closest point
      if (move_to_avoid_obstacle != no_avoidance) {
        if (lidar_data.ranges[i] < critical_distance) {
          critical_distance = lidar_data.ranges[i];
          if (move_to_avoid_obstacle == left) {
            // turn with x speed where x is dependent on how close the point is
            ang_velocity =
                (min_critical_distance - critical_distance) * ang_vel_gain;

          } else {
            ang_velocity =
                -(min_critical_distance - critical_distance) * ang_vel_gain;
          }
        }
      }
    }
  }
  return ang_velocity;
}

void ObstacleAvoidance::lidarCallback(
    const sensor_msgs::LaserScan::ConstPtr &laser_scan_message) {
  // get the lidar data from the subscribed topic
  lidar_data.angle_min = laser_scan_message->angle_min;
  lidar_data.angle_max = laser_scan_message->angle_max;
  lidar_data.angle_increment = laser_scan_message->angle_increment;

  int lidar_array_size = laser_scan_message->ranges.size();

  lidar_data.ranges.resize(lidar_array_size);

  for (int i = 0; i < lidar_array_size; i++) {
    lidar_data.ranges[i] = laser_scan_message->ranges[i];
  }
  
  double ang_velocity = 0;
  bool started_on_left = true;
  bool first_action = true;
  bool first_period_no_obs = true;

  ang_velocity = angVelocityToAvoidCollision();

  if ((ang_velocity != 0) && (first_action == true)) {
    first_period_no_obs = false;
    if (ang_velocity > 0) {
      started_on_left = false;
	} else if (ang_velocity <= 0) {
	  started_on_left = true;
	}
  } else if ((ang_velocity == 0) && (first_period_no_obs == false)) {
	first_action = false;
	if (started_on_left == false) {
	  ang_velocity = -wall_hug_vel;
	} else {
	  ang_velocity = wall_hug_vel;
	}
  }
  
  setAngVelMsg(ang_velocity);
  publishVel();
}

void ObstacleAvoidance::publishVel() { twist_publisher.publish(vel_msg); }

void ObstacleAvoidance::setLinearVelMsg(float linear_vel) {
  vel_msg.linear.x = linear_vel;
}

void ObstacleAvoidance::setAngVelMsg(float ang_vel) {
  vel_msg.angular.z = ang_vel;
}

float ObstacleAvoidance::getLinearVelMsg() { return vel_msg.linear.x; }

float ObstacleAvoidance::getAngVelMsg() { return vel_msg.angular.z; }
