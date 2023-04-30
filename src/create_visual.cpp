#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "create_visual");
  ros::NodeHandle n;
  ros::Rate r(2);
  ros::Publisher ellipsis_pub = n.advertise<visualization_msgs::Marker>("spacecraft/ellipsis", 1);
  ros::Publisher central_body_pub = n.advertise<visualization_msgs::Marker>("spacecraft/central_body", 1);
  ros::Publisher spacecraft_body_pub = n.advertise<visualization_msgs::Marker>("spacecraft/spacecraft_body", 1);

  // Set our initial shape type to be a cube
  uint32_t shape_line_lst = visualization_msgs::Marker::LINE_LIST;
  uint32_t shape_sphere = visualization_msgs::Marker::SPHERE;


  // Set dimensions of ellipse
  float a_orbit = 15;
  float b_orbit = 9;
  float r_pe = 8;
  float theta = M_PI/4;

  while (ros::ok())
  {
    visualization_msgs::Marker ellipsis;
    visualization_msgs::Marker central_body;
    visualization_msgs::Marker spacecraft_body;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    ellipsis.header.frame_id = "visual_frame";
    ellipsis.header.stamp = ros::Time::now();

    central_body.header.frame_id = "visual_frame";
    central_body.header.stamp = ros::Time::now();

    spacecraft_body.header.frame_id = "visual_frame";
    spacecraft_body.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    ellipsis.ns = "orbit";
    ellipsis.id = 0;

    central_body.ns = "orbit";
    central_body.id = 1;

    spacecraft_body.ns = "orbit";
    spacecraft_body.id = 2;


    // Set the marker type.
    ellipsis.type = shape_line_lst;
    central_body.type = shape_sphere;
    spacecraft_body.type = shape_sphere;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    ellipsis.action = visualization_msgs::Marker::ADD;
    central_body.action = visualization_msgs::Marker::ADD;
    spacecraft_body.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    ellipsis.pose.position.x = 0;
    ellipsis.pose.position.y = 0;
    ellipsis.pose.position.z = 0;
    ellipsis.pose.orientation.x = 0.0;
    ellipsis.pose.orientation.y = 0.0;
    ellipsis.pose.orientation.z = 0.0;
    ellipsis.pose.orientation.w = 1.0;

    central_body.pose.position.x = 0;
    central_body.pose.position.y = 0;
    central_body.pose.position.z = 0;
    central_body.pose.orientation.x = 0.0;
    central_body.pose.orientation.y = 0.0;
    central_body.pose.orientation.z = 0.0;
    central_body.pose.orientation.w = 1.0;

    spacecraft_body.pose.position.x = b_orbit*cos(theta);
    spacecraft_body.pose.position.y = a_orbit*(sin(theta)+1) - r_pe;
    spacecraft_body.pose.position.z = 0;
    spacecraft_body.pose.orientation.x = 0.0;
    spacecraft_body.pose.orientation.y = 0.0;
    spacecraft_body.pose.orientation.z = 0.0;
    spacecraft_body.pose.orientation.w = 1.0;


    // Set the scale of the marker
    ellipsis.scale.x = 0.1;

    central_body.scale.x = 3.0;
    central_body.scale.y = 3.0;
    central_body.scale.z = 3.0;

    spacecraft_body.scale.x = 1.0;
    spacecraft_body.scale.y = 1.0;
    spacecraft_body.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    ellipsis.color.r = 0.0f;
    ellipsis.color.g = 1.0f;
    ellipsis.color.b = 0.0f;
    ellipsis.color.a = 1.0;

    central_body.color.r = 1.0f;
    central_body.color.g = 0.0f;
    central_body.color.b = 0.0f;
    central_body.color.a = 1.0;

    spacecraft_body.color.r = 0.0f;
    spacecraft_body.color.g = 0.0f;
    spacecraft_body.color.b = 1.0f;
    spacecraft_body.color.a = 1.0;


    ellipsis.lifetime = ros::Duration();
    central_body.lifetime = ros::Duration();
    spacecraft_body.lifetime = ros::Duration();

    // Create the vertices for the lines
    for (uint32_t i = 0; i < 1000; ++i)
    {
      float x = b_orbit * cos(i / 1000.0f * 2 * M_PI);
      float y = a_orbit * (sin(i / 1000.0f * 2 * M_PI) + 1) - r_pe;

      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      p.z = 0;

      ellipsis.points.push_back(p);
    }

    // Publish the marker
    while (ellipsis_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    ellipsis_pub.publish(ellipsis);
    central_body_pub.publish(central_body);
    spacecraft_body_pub.publish(spacecraft_body);

    r.sleep();
  }
}