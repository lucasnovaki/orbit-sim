#include <ros/ros.h>
#include "orbit_sim/create_visual.h"

#define VISUAL_NODE_RATE 20


// Define subscriber for target orbit marker

int main( int argc, char** argv )
{
  // Initialize node
  ros::init(argc, argv, "create_visual");
  ros::NodeHandle n;
  ros::Rate r(VISUAL_NODE_RATE);

  //Setup drawer instance
  Drawer drawer;
  drawer.SetUp(n);

  while (ros::ok())
  {

    // Publish the marker
    while (drawer.ellipsis_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    ros::spinOnce();
    drawer.PublishMarkers();

    r.sleep();
  }
}