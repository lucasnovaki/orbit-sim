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

  while (ros::ok())
  {

    //Setup
    Drawer drawer;
    drawer.SetUp(n);

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
    drawer.ellipsis_pub.publish(drawer.currentEllipsis);
    drawer.tar_ellipsis_pub.publish(drawer.targetEllipsis);
    drawer.central_body_pub.publish(drawer.central_body);
    drawer.spacecraft_body_pub.publish(drawer.spacecraft_body);
    drawer.spacecraft_vel_pub.publish(drawer.spacecraft_velocity);

    r.sleep();
  }
}