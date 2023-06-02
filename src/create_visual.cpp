#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <orbit_sim/State2d.h>
#include <orbit_sim/Orbit2d.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

#define SCALE_FACTOR 0.001
#define VISUAL_NODE_RATE 20

//Define global vars
visualization_msgs::Marker currentEllipsis;
visualization_msgs::Marker targetEllipsis;
visualization_msgs::Marker central_body;
visualization_msgs::Marker spacecraft_body;
visualization_msgs::Marker spacecraft_velocity;

geometry_msgs::Point rotate2dPoint(geometry_msgs::Point p_in, float angle){
    geometry_msgs::Point p_out;
    p_out.x = cos(angle)*p_in.x - sin(angle)*p_in.y;
    p_out.y = sin(angle)*p_in.x + cos(angle)*p_in.y;
    p_out.z = 0;
    return p_out;
}

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

void drawEllipsis(const orbit_sim::Orbit2d::ConstPtr& orbitParams, visualization_msgs::Marker& ellipsisObj){
    // Get update values for orbit params
    float current_a_orbit = orbitParams->a_orbit;
    float current_e_orbit = orbitParams->e_orbit;
    float current_b_orbit = current_a_orbit*sqrt(1 - pow(current_e_orbit,2));

    if (current_e_orbit <= 1 & current_e_orbit > 0){
      
      //calculate orbit shape only if its indeed ellipsis
      float r_pe = current_a_orbit*(1 - current_e_orbit);
        
      // Create the vertices for the lines
      for (uint32_t i = 0; i < 1000; ++i)
      {
        float x_i = current_b_orbit * cos(i / 1000.0f * 2 * M_PI);
        float y_i = current_a_orbit * (sin(i / 1000.0f * 2 * M_PI) + 1) - r_pe;

        geometry_msgs::Point p;
        p.x = SCALE_FACTOR*x_i;
        p.y = SCALE_FACTOR*y_i;
        p.z = 0;

        ellipsisObj.points[i] = rotate2dPoint(p, -orbitParams->w_orbit + M_PI/2);
      }
    }

    return;
}

void CurrentOrbitCallback(const orbit_sim::Orbit2d::ConstPtr& orbitParams){
    drawEllipsis(orbitParams, currentEllipsis);
}

void TargetOrbitCallback(const orbit_sim::Orbit2d::ConstPtr& orbitParams){
    drawEllipsis(orbitParams, targetEllipsis);
}

// Define subscriber for target orbit marker

int main( int argc, char** argv )
{
  // Initialize node
  ros::init(argc, argv, "create_visual");
  ros::NodeHandle n;
  ros::Rate r(VISUAL_NODE_RATE);

  //Set-up publishers for rviz
  ros::Publisher ellipsis_pub = n.advertise<visualization_msgs::Marker>("spacecraft/ellipsis", 1);
  ros::Publisher tar_ellipsis_pub = n.advertise<visualization_msgs::Marker>("spacecraft/target_ellipsis", 1);
  ros::Publisher central_body_pub = n.advertise<visualization_msgs::Marker>("spacecraft/central_body", 1);
  ros::Publisher spacecraft_body_pub = n.advertise<visualization_msgs::Marker>("spacecraft/spacecraft_body", 1);
  ros::Publisher spacecraft_vel_pub = n.advertise<visualization_msgs::Marker>("spacecraft/spacecraft_velocity", 1);

  //Set-up subscriber
  ros::Subscriber spacecraft_pos_sub = n.subscribe("/simulation_data/states", 1, PosCallback);
  ros::Subscriber ellipsis_sub = n.subscribe("/simulation_data/orbit_params", 1, CurrentOrbitCallback);
  ros::Subscriber tar_ellipsis_sub = n.subscribe("/navigation/target_orbit_params", 1, TargetOrbitCallback);

  //dimensions of ellipse
  float theta = 0;
  const float r_pe = 15150.0; //km
  const float r_apo= 56070.0; //km
  float a_orbit = (r_pe + r_apo)/2; //km
  float b_orbit = 0.7*a_orbit;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  currentEllipsis.header.frame_id = "visual_frame";
  currentEllipsis.header.stamp = ros::Time::now();

  targetEllipsis.header.frame_id = "visual_frame";
  targetEllipsis.header.stamp = ros::Time::now();

  central_body.header.frame_id = "visual_frame";
  central_body.header.stamp = ros::Time::now();

  spacecraft_body.header.frame_id = "visual_frame";
  spacecraft_body.header.stamp = ros::Time::now();

  spacecraft_velocity.header.frame_id = "visual_frame";
  spacecraft_velocity.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  currentEllipsis.ns = "orbit";
  currentEllipsis.id = 0;

  central_body.ns = "orbit";
  central_body.id = 1;

  spacecraft_body.ns = "orbit";
  spacecraft_body.id = 2;

  spacecraft_velocity.ns = "orbit";
  spacecraft_velocity.id = 3;

  targetEllipsis.ns = "orbit";
  targetEllipsis.id = 4;

  // Set the marker type.
  currentEllipsis.type = visualization_msgs::Marker::LINE_LIST;
  targetEllipsis.type = visualization_msgs::Marker::LINE_LIST;
  central_body.type = visualization_msgs::Marker::SPHERE;
  spacecraft_body.type = visualization_msgs::Marker::SPHERE;
  spacecraft_velocity.type = visualization_msgs::Marker::ARROW;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  currentEllipsis.action = visualization_msgs::Marker::ADD;
  targetEllipsis.action = visualization_msgs::Marker::ADD;
  central_body.action = visualization_msgs::Marker::ADD;
  spacecraft_body.action = visualization_msgs::Marker::ADD;
  spacecraft_velocity.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  currentEllipsis.pose.position.x = 0;
  currentEllipsis.pose.position.y = 0;
  currentEllipsis.pose.position.z = 0;
  currentEllipsis.pose.orientation.x = 0.0;
  currentEllipsis.pose.orientation.y = 0.0;
  currentEllipsis.pose.orientation.z = 0.0;
  currentEllipsis.pose.orientation.w = 1.0;

  targetEllipsis.pose.position.x = 0;
  targetEllipsis.pose.position.y = 0;
  targetEllipsis.pose.position.z = 0;
  targetEllipsis.pose.orientation.x = 0.0;
  targetEllipsis.pose.orientation.y = 0.0;
  targetEllipsis.pose.orientation.z = 0.0;
  targetEllipsis.pose.orientation.w = 1.0;

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
  currentEllipsis.scale.x = 0.1;
  targetEllipsis.scale.x = 0.1;

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
  currentEllipsis.color.r = 0.0f;
  currentEllipsis.color.g = 1.0f;
  currentEllipsis.color.b = 0.0f;
  currentEllipsis.color.a = 1.0;

  targetEllipsis.color.r = 1.0f;
  targetEllipsis.color.g = 0.0f;
  targetEllipsis.color.b = 1.0f;
  targetEllipsis.color.a = 1.0;

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


  currentEllipsis.lifetime = ros::Duration();
  targetEllipsis.lifetime = ros::Duration(); 
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

    currentEllipsis.points.push_back(p);
    targetEllipsis.points.push_back(p);
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
    ellipsis_pub.publish(currentEllipsis);
    tar_ellipsis_pub.publish(targetEllipsis);
    central_body_pub.publish(central_body);
    spacecraft_body_pub.publish(spacecraft_body);
    spacecraft_vel_pub.publish(spacecraft_velocity);

    r.sleep();
  }
}