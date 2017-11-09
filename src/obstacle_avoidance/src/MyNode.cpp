/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <MyNode.h>

sensor_msgs::LaserScan AvoidObstacleClass::lidar_data;

int sizeOfLidarArray(sensor_msgs::LaserScan lidar);

AvoidObstacleClass::AvoidObstacleClass() {
  // initialise velocity
  vel_msg.linear.x = LINEAR_VEL;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;
}

AvoidObstacleClass::AvoidObstacleClass(int argc, char **argv,
                                       std::string node_name) {
  // Setup NodeHandles
  ros::init(argc, argv, node_name);
  nh = new ros::NodeHandle;

  // Setup Subscriber/Publisher
  twist_publisher = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  laser_scan_subscriber =
      nh->subscribe("/scan", 10, &AvoidObstacleClass::lidarCallback, this);

  // initialise velocity
  vel_msg.linear.x = LINEAR_VEL;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;
  std::cout << "completed construction \n";
  // std::cout << "lidar data " << lidar_data.angle_max << std::endl;
  // std::cout << "lidar size " << sizeOfLidarArray(lidar_data) << std::endl;
  // std::cout << lidar_data << std::endl;

  /* Obtains character from the parameter server (or launch file), sets '!' as
default
std::string parameter_name = "my_node/character";
std::string default_character = "!";
SB_getParam(nh, parameter_name, suffix, default_character);

// Setup Subscriber(s)
std::string topic_to_subscribe_to = "subscribe_topic";
int refresh_rate = 10;
my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate,
&MyClass::subscriberCallBack, this);

// Setup Publisher(s)
std::string topic = private_nh.resolveName("publish_topic");
uint32_t queue_size = 1;
my_publisher = private_nh.advertise<std_msgs::String>(topic, queue_size);
  */
}

AvoidObstacleClass::~AvoidObstacleClass() {
  delete nh;
  std::cout << "completed destruction \n";
}

avoidance AvoidObstacleClass::moveToAvoidCollision(float lidar_distance,
                                                   float lidar_angle) {
  avoidance move_to_avoid_collision;
  if ((lidar_angle < M_PI / 2) && (lidar_angle > -M_PI / 2)) {
    float x = lidar_distance * sin(lidar_angle);
    // double y = lidar_distance*cos(lidar_angle);

    if ((x < VEHICLE_WIDTH / 2) && (x > -VEHICLE_WIDTH / 2)) {
      if (x <= 0) {
        move_to_avoid_collision = left;
        // std::cout << "left" << std::endl;
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

bool AvoidObstacleClass::isLeftSideOfGoal() {
  // find out if you are on the left or right side of the map
  // make sure lidar_array_size is greater than 61
  int lidar_array_size = sizeOfLidarArray(lidar_data);
  bool left_side = TRUE; // should make an enum set as
  int x = 1;
  int i = 30;

  if (lidar_array_size != 0) {

    while ((lidar_data.ranges[i] == NAN) || (lidar_data.ranges[i] <= 0.03) ||
           (lidar_data.ranges[i] >= 5) && (i < (lidar_array_size - 30))) {
      i++;
    }
    if (i != (lidar_array_size - 30)) {
      i++;
      while (i < (lidar_array_size - 30)) {
        if ((lidar_data.ranges[i] != NAN) && (lidar_data.ranges[i] > 0.03) &&
            (lidar_data.ranges[i] < 5)) {
          double a = lidar_data.ranges[i];
          double b = lidar_data.ranges[i - x];
          double C = lidar_data.angle_increment;
          // Cosine Rule
          double dist_between_pts =
              sqrt(pow(a, 2) + pow(b, 2) - 2 * a * b * cos(C));
          if ((dist_between_pts > MIN_GOAL_WIDTH) &&
              (dist_between_pts < MAX_GOAL_WIDTH)) {
            if (i < (lidar_array_size / 2 - 10)) {
              left_side = FALSE;
            } else if (i > (lidar_array_size / 2 + 10)) {
              left_side = TRUE;
            }
          }
          x = 1;
        } else {
          x++;
        }
        i++;
      }
    }
  }

  return left_side;
}

double AvoidObstacleClass::angVelocityToAvoidCollision() {

  double critical_distance = MIN_CRITICAL_DISTANCE;
  double ang_velocity = 0;
  avoidance move_to_avoid_obstacle;
  // std::cerr << "angV 1st action" << std::endl;

  for (int i = 0; i < sizeOfLidarArray(lidar_data); i++) {
    if ((lidar_data.ranges[i] != NAN) && (lidar_data.ranges[i] > 0.03) &&
        (lidar_data.ranges[i] < 5)) {
      // check if lidar point is going to collide
      // std::cerr << "angV 2nd action" << std::endl;
      move_to_avoid_obstacle = moveToAvoidCollision(
          lidar_data.ranges[i],
          (lidar_data.angle_min + lidar_data.angle_increment * i));
      // std::cerr << "angV 3rd action" << std::endl;
      // if most immediate threat/closest point
      if (move_to_avoid_obstacle != no_avoidance) {
        if (lidar_data.ranges[i] < critical_distance) {
          critical_distance = lidar_data.ranges[i];
          if (move_to_avoid_obstacle == left) {
            // turn with x speed where x is dependent on how close the point is
            ang_velocity =
                (MIN_CRITICAL_DISTANCE - critical_distance) * ANG_VEL_GAIN;
            // std::cout << "turn left " << ang_velocity << std::endl;

          } else {
            ang_velocity =
                -(MIN_CRITICAL_DISTANCE - critical_distance) * ANG_VEL_GAIN;
            // std::cout << "turn right " << ang_velocity << std::endl;
          }
        }
      } else {
        // std::cout << "no avoidance" << std::endl;
      }
    }
  }
  std::cout << "ang vel " << ang_velocity << std::endl;
  return ang_velocity;
}

void AvoidObstacleClass::lidarCallback(
    const sensor_msgs::LaserScan::ConstPtr &laser_scan_message) {
  // get the lidar data from the subscribed topic
  lidar_data.angle_min = laser_scan_message->angle_min;
  lidar_data.angle_max = laser_scan_message->angle_max;
  lidar_data.angle_increment = laser_scan_message->angle_increment;
  // std::cout << "lidar data " << lidar_data.angle_max << std::endl;
  // std::cout << "lidar scan " << laser_scan_message->angle_max << std::endl;

  int lidar_array_size = sizeOfLidarArray(lidar_data);

  lidar_data.ranges.resize(lidar_array_size);

  for (int i = 0; i < lidar_array_size; i++) {
    lidar_data.ranges[i] = laser_scan_message->ranges[i]; // edit
  }
}

void AvoidObstacleClass::publishVel() { twist_publisher.publish(vel_msg); }

void AvoidObstacleClass::setLinearVelMsg(float linear_vel) {
  vel_msg.linear.x = linear_vel;
}

void AvoidObstacleClass::setAngVelMsg(float ang_vel) {
  vel_msg.angular.z = ang_vel;
}

float AvoidObstacleClass::getLinearVelMsg() { return vel_msg.linear.x; }

float AvoidObstacleClass::getAngVelMsg() { return vel_msg.angular.z; }

int sizeOfLidarArray(sensor_msgs::LaserScan lidar) {
  float current_angle = lidar.angle_min;
  int i = 0;
  int lidar_array_size;
  if ((lidar.angle_min == 0) && (lidar.angle_max == 0)) {
    lidar_array_size = 0;
  } else {
    while (current_angle < lidar.angle_max) {
      current_angle += lidar.angle_increment;
      i++;
    }

    lidar_array_size = i + 1;
  }

  return lidar_array_size;
}
