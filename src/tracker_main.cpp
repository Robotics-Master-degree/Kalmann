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
  while (ros::ok()) //( )
  {
        //execute pending callbacks
        ros::spinOnce();
        if (!tracker.new_detection)
        {
          std::cout << "Entered" << std::endl;
          tracker.prediction(ros::Time::now());
          tracker.publish();
          tracker.new_detection=false;
        }
        //prediction()
        //tracker.new_detection=false;
        std::cout << "new_detection: " << tracker.new_detection << std::endl;
        loopRate.sleep();
  }

  //exit program
  return 0;

}
