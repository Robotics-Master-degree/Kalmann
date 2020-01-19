#include "tracker_node.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <rate.h>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  //init ros
  ros::init(argc, argv, "tracker_node");

  //create ros wrapper object
  TrackerNode tracker;

  //set node loop rate
  ros::Rate loopRate(tracker.getRate());

  //node loop
  while ( ros::ok() )
  {
        //execute pending callbacks
        ros::spinOnce();

        //prediction()

        //do things

        tracker.prediction();

        geometry_msgs::Vector3 kf;
        kf.x=tracker.x_t_[0];
        kf.y=tracker.x_t_[1];
        kf.z=tracker.radius;
        tracker.Kalmann_Filter_.publish(kf);
        //tracker.correction();
        //imgp.process();

        //publish things
        //imgp.publishImage();
        //imgp.publishMarker();
        //imgp.getMarker();
        //imgp.stateSpace();
        //relax to fit output rate
        loopRate.sleep();
  }

  //exit program
  return 0;

}
