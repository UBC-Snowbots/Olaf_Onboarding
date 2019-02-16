#include <GoThroughHole.h>

geometry_msgs::Point32 findHole (sensor_msgs::PointCloud scanData) {
    // finds the set of coordinates to go through
    /*
      determine the slope of the line through consecutive points

      then compare the consecutive slopes of the points

      if this difference is bigger than a certain threshold then mark this
          point as the beggining of the hole
      
      iterate through consecutive points again to find end of hole

      then average the two to find the center
    */
}

void moveThroughHole (geometry_msgs::Point32 centerOfHole ) {
    /* Use proportional control to lead robot to hole
       
       set the Twist message to be proportional to the distance to the
       center of the hole
       
       set the Twist message to also have an angle proportional to the
       angular separation from the center of the hole
       
       update the message as the robot moves
    */
    /*
      Problem: Account for the fact that the robot is moving
    */
}
