#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <orbit_sim/State2d.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>

#define SCALE_FACTOR 0.001
#define VISUAL_NODE_RATE 20

//Define global vars
visualization_msgs::Marker ellipsis;
visualization_msgs::Marker central_body;
visualization_msgs::Marker spacecraft_body;
visualization_msgs::Marker spacecraft_velocity;

void updateVelocityArrow(geometry_msgs::Vector3 pos, geometry_msgs::Vector3 vel){
    geometry_msgs::Point arrowBegin;
    geometry_msgs::Point arrowEnd;

    arrowBegin.x = SCALE_FACTOR*pos.x;
    arrowBegin.y = SCALE_FACTOR*pos.y;
    arrowBegin.z = 0;
    arrowEnd.x = vel.x + arrowBegin.x;
    arrowEnd.y = vel.y + arrowBegin.y;
    arrowEnd.z = 0;

    spacecraft_velocity.points[0] = arrowBegin;
    spacecraft_velocity.points[1] = arrowEnd;

    return;
}

void updateSpacecraftBody(geometry_msgs::Vector3 pos){
  spacecraft_body.pose.position.x = SCALE_FACTOR*pos.x;
  spacecraft_body.pose.position.y = SCALE_FACTOR*pos.y;
  return;
}

void PosCallback(const orbit_sim::State2d::ConstPtr& state){
    
    geometry_msgs::Vector3 statePos = state->position;
    geometry_msgs::Vector3 stateVel = state->velocity;

    updateSpacecraftBody(statePos);
    updateVelocityArrow(statePos, stateVel);

    return;
}

void OrbitCallback(const orbit_sim::State2d::ConstPtr& state){
    //change data type later!
    return;
}

int main( int argc, char** argv )
{
  // Initialize node
  ros::init(argc, argv, "create_visual");
  ros::NodeHandle n;
  ros::Rate r(VISUAL_NODE_RATE);

  //Set-up publishers for rviz
  ros::Publisher ellipsis_pub = n.advertise<visualization_msgs::Marker>("spacecraft/ellipsis", 1);
  ros::Publisher central_body_pub = n.advertise<visualization_msgs::Marker>("spacecraft/central_body", 1);
  ros::Publisher spacecraft_body_pub = n.advertise<visualization_msgs::Marker>("spacecraft/spacecraft_body", 1);
  ros::Publisher spacecraft_vel_pub = n.advertise<visualization_msgs::Marker>("spacecraft/spacecraft_velocity", 1);

  //Set-up subscriber
  ros::Subscriber spacecraft_pos_sub = n.subscribe("/simulation_data/states", 20, PosCallback);
  ros::Subscriber ellipsis_sub = n.subscribe("/simulation_data/orbit_params", 1, OrbitCallback);

  //dimensions of ellipse
  float theta = 0;
  const float r_pe = 15150.0; //km
  const float r_apo= 56070.0; //km
  float a_orbit = (r_pe + r_apo)/2; //km
  float b_orbit = 0.7*a_orbit;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  ellipsis.header.frame_id = "visual_frame";
  ellipsis.header.stamp = ros::Time::now();

  central_body.header.frame_id = "visual_frame";
  central_body.header.stamp = ros::Time::now();

  spacecraft_body.header.frame_id = "visual_frame";
  spacecraft_body.header.stamp = ros::Time::now();

  spacecraft_velocity.header.frame_id = "visual_frame";
  spacecraft_velocity.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  ellipsis.ns = "orbit";
  ellipsis.id = 0;

  central_body.ns = "orbit";
  central_body.id = 1;

  spacecraft_body.ns = "orbit";
  spacecraft_body.id = 2;

  spacecraft_velocity.ns = "orbit";
  spacecraft_velocity.id = 3;

  // Set the marker type.
  ellipsis.type = visualization_msgs::Marker::LINE_LIST;
  central_body.type = visualization_msgs::Marker::SPHERE;
  spacecraft_body.type = visualization_msgs::Marker::SPHERE;
  spacecraft_velocity.type = visualization_msgs::Marker::ARROW;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  ellipsis.action = visualization_msgs::Marker::ADD;
  central_body.action = visualization_msgs::Marker::ADD;
  spacecraft_body.action = visualization_msgs::Marker::ADD;
  spacecraft_velocity.action = visualization_msgs::Marker::ADD;

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

  spacecraft_body.pose.position.x = SCALE_FACTOR*b_orbit*sin(theta);
  spacecraft_body.pose.position.y = SCALE_FACTOR*(a_orbit*(1-cos(theta)) - r_pe);
  spacecraft_body.pose.position.z = 0;
  spacecraft_body.pose.orientation.x = 0.0;
  spacecraft_body.pose.orientation.y = 0.0;
  spacecraft_body.pose.orientation.z = 0.0;
  spacecraft_body.pose.orientation.w = 1.0;


  // Set the scale of the marker
  ellipsis.scale.x = 0.1;

  central_body.scale.x = SCALE_FACTOR*13000;
  central_body.scale.y = SCALE_FACTOR*13000;
  central_body.scale.z = SCALE_FACTOR*13000;

  spacecraft_body.scale.x = 2;
  spacecraft_body.scale.y = 2;
  spacecraft_body.scale.z = 2;

  spacecraft_velocity.scale.x = 0.2;
  spacecraft_velocity.scale.y = 0.2;
  spacecraft_velocity.scale.z = 0.4;

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

  spacecraft_velocity.color.r = 1.0f;
  spacecraft_velocity.color.g = 0.0f;
  spacecraft_velocity.color.b = 0.0f;
  spacecraft_velocity.color.a = 1.0;


  ellipsis.lifetime = ros::Duration();
  central_body.lifetime = ros::Duration();
  spacecraft_body.lifetime = ros::Duration();

  // Create the vertices for the lines
  for (uint32_t i = 0; i < 1000; ++i)
  {
    float x = b_orbit * cos(i / 1000.0f * 2 * M_PI);
    float y = a_orbit * (sin(i / 1000.0f * 2 * M_PI) + 1) - r_pe;

    geometry_msgs::Point p;
    p.x = SCALE_FACTOR*x;
    p.y = SCALE_FACTOR*y;
    p.z = 0;

    ellipsis.points.push_back(p);
  }

  //create initial velocity arrow
  geometry_msgs::Point arrowBegin = spacecraft_body.pose.position;

  geometry_msgs::Point arrowEnd;
  arrowEnd.x = arrowBegin.x + 6;
  arrowEnd.y = 0;
  arrowEnd.z = 0;

  spacecraft_velocity.points.push_back(arrowBegin);
  spacecraft_velocity.points.push_back(arrowEnd);

  while (ros::ok())
  {

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

    ros::spinOnce();
    ellipsis_pub.publish(ellipsis);
    central_body_pub.publish(central_body);
    spacecraft_body_pub.publish(spacecraft_body);
    spacecraft_vel_pub.publish(spacecraft_velocity);

    r.sleep();
  }
}