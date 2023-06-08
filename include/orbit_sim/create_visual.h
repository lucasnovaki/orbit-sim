#ifndef CREATE_VISUAL_H
#define CREATE_VISUAL_H

#include <ros/ros.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <orbit_sim/State2d.h>
#include <orbit_sim/Orbit2d.h>
#include <geometry_msgs/Vector3.h>

#define SCALE_FACTOR 0.001


//Auxiliary Functions

geometry_msgs::Point rotate2dPoint(geometry_msgs::Point p_in, float angle){
    geometry_msgs::Point p_out;
    p_out.x = cos(angle)*p_in.x - sin(angle)*p_in.y;
    p_out.y = sin(angle)*p_in.x + cos(angle)*p_in.y;
    p_out.z = 0;
    return p_out;
}

// Drawer class
 class Drawer{
    public:

        // Publishers/Subscribers
        ros::Publisher ellipsis_pub;
        ros::Publisher tar_ellipsis_pub;
        ros::Publisher central_body_pub;
        ros::Publisher spacecraft_body_pub;
        ros::Publisher spacecraft_vel_pub;

        ros::Subscriber spacecraft_pos_sub;
        ros::Subscriber ellipsis_sub;
        ros::Subscriber tar_ellipsis_sub;

        //Markers
        visualization_msgs::Marker currentEllipsis;
        visualization_msgs::Marker targetEllipsis;
        visualization_msgs::Marker central_body;
        visualization_msgs::Marker spacecraft_body;
        visualization_msgs::Marker spacecraft_velocity;

        // Main publisher method
        void PublishMarkers();

        //Callback methods
        void updateVelocityArrow(geometry_msgs::Vector3, geometry_msgs::Vector3);
        void updateSpacecraftBody(geometry_msgs::Vector3);
        void PosCallback(const orbit_sim::State2d::ConstPtr&);
        void CurrentOrbitCallback(const orbit_sim::Orbit2d::ConstPtr&);
        void TargetOrbitCallback(const orbit_sim::Orbit2d::ConstPtr&);

        //Auxiliary methods
        void drawEllipsis(const orbit_sim::Orbit2d::ConstPtr&, visualization_msgs::Marker&);

        //Setup methods
        void MarkerSetUp();
        void MarkerBasicSetUp(visualization_msgs::Marker&, int);
        void CommsSetUp(ros::NodeHandle&);
        void SetColors();
        void SetScales();

        void SetUp(ros::NodeHandle& n){
            this->CommsSetUp(n);
            this->MarkerSetUp();
        }
 };

 void Drawer::updateVelocityArrow(geometry_msgs::Vector3 pos, geometry_msgs::Vector3 vel){
    geometry_msgs::Point arrowBegin;
    geometry_msgs::Point arrowEnd;

    arrowBegin.x = SCALE_FACTOR*pos.x;
    arrowBegin.y = SCALE_FACTOR*pos.y;
    arrowBegin.z = 0;
    arrowEnd.x = vel.x + arrowBegin.x;
    arrowEnd.y = vel.y + arrowBegin.y;
    arrowEnd.z = 0;

    this->spacecraft_velocity.points[0] = arrowBegin;
    this->spacecraft_velocity.points[1] = arrowEnd;

    return;
}

void Drawer::updateSpacecraftBody(geometry_msgs::Vector3 pos){
  this->spacecraft_body.pose.position.x = SCALE_FACTOR*pos.x;
  this->spacecraft_body.pose.position.y = SCALE_FACTOR*pos.y;
  return;
}

void Drawer::drawEllipsis(const orbit_sim::Orbit2d::ConstPtr& orbitParams, visualization_msgs::Marker& ellipsisObj){
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

void Drawer::PosCallback(const orbit_sim::State2d::ConstPtr& state){
    
    geometry_msgs::Vector3 statePos = state->position;
    geometry_msgs::Vector3 stateVel = state->velocity;

    Drawer::updateSpacecraftBody(statePos);
    Drawer::updateVelocityArrow(statePos, stateVel);

    return;
}

void Drawer::CurrentOrbitCallback(const orbit_sim::Orbit2d::ConstPtr& orbitParams){
    Drawer::drawEllipsis(orbitParams, this->currentEllipsis);
}

void Drawer::TargetOrbitCallback(const orbit_sim::Orbit2d::ConstPtr& orbitParams){
    Drawer::drawEllipsis(orbitParams, this->targetEllipsis);
    Drawer::MarkerBasicSetUp(this->targetEllipsis, 1);
}

void Drawer::PublishMarkers(){
    this->ellipsis_pub.publish(this->currentEllipsis);
    this->tar_ellipsis_pub.publish(this->targetEllipsis);
    this->central_body_pub.publish(this->central_body);
    this->spacecraft_body_pub.publish(this->spacecraft_body);
    this->spacecraft_vel_pub.publish(this->spacecraft_velocity);
    return;
}

void Drawer::CommsSetUp(ros::NodeHandle& n){
    //Publishers for rviz
    this->ellipsis_pub = n.advertise<visualization_msgs::Marker>("spacecraft/ellipsis", 1);
    this->tar_ellipsis_pub = n.advertise<visualization_msgs::Marker>("spacecraft/target_ellipsis", 1);
    this->central_body_pub = n.advertise<visualization_msgs::Marker>("spacecraft/central_body", 1);
    this->spacecraft_body_pub = n.advertise<visualization_msgs::Marker>("spacecraft/spacecraft_body", 1);
    this->spacecraft_vel_pub = n.advertise<visualization_msgs::Marker>("spacecraft/spacecraft_velocity", 1);
    
    //Subscribers
    this->spacecraft_pos_sub = n.subscribe("/simulation_data/states", 1, &Drawer::PosCallback, this);
    this->ellipsis_sub = n.subscribe("/simulation_data/orbit_params", 1, &Drawer::CurrentOrbitCallback, this);
    this->tar_ellipsis_sub = n.subscribe("/navigation/target_orbit_params", 1, &Drawer::TargetOrbitCallback, this);

    return;
}

void Drawer::MarkerBasicSetUp(visualization_msgs::Marker& marker, int id){
    marker.header.frame_id = "visual_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "orbit";
    marker.id = id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    return;
}

void Drawer::SetColors(){
    // Set the color -- be sure to set alpha to something non-zero!
    this->currentEllipsis.color.r = 0.0f;
    this->currentEllipsis.color.g = 1.0f;
    this->currentEllipsis.color.b = 0.0f;
    this->currentEllipsis.color.a = 1.0;

    this->targetEllipsis.color.r = 1.0f;
    this->targetEllipsis.color.g = 0.0f;
    this->targetEllipsis.color.b = 1.0f;
    this->targetEllipsis.color.a = 1.0;

    this->central_body.color.r = 1.0f;
    this->central_body.color.g = 0.0f;
    this->central_body.color.b = 0.0f;
    this->central_body.color.a = 1.0;

    this->spacecraft_body.color.r = 0.0f;
    this->spacecraft_body.color.g = 0.0f;
    this->spacecraft_body.color.b = 1.0f;
    this->spacecraft_body.color.a = 1.0;

    this->spacecraft_velocity.color.r = 1.0f;
    this->spacecraft_velocity.color.g = 0.0f;
    this->spacecraft_velocity.color.b = 0.0f;
    this->spacecraft_velocity.color.a = 1.0;
}

void Drawer::SetScales(){
    // Set the scale of the marker
    this->currentEllipsis.scale.x = 0.1;
    this->targetEllipsis.scale.x = 0.1;

    this->central_body.scale.x = SCALE_FACTOR*13000;
    this->central_body.scale.y = SCALE_FACTOR*13000;
    this->central_body.scale.z = SCALE_FACTOR*13000;

    this->spacecraft_body.scale.x = 2;
    this->spacecraft_body.scale.y = 2;
    this->spacecraft_body.scale.z = 2;

    this->spacecraft_velocity.scale.x = 0.2;
    this->spacecraft_velocity.scale.y = 0.2;
    this->spacecraft_velocity.scale.z = 0.4;
}

void Drawer::MarkerSetUp(){
  // Basic Setup
  Drawer::MarkerBasicSetUp(this->currentEllipsis, 0);
  Drawer::MarkerBasicSetUp(this->targetEllipsis, 1);
  Drawer::MarkerBasicSetUp(this->central_body, 2);
  Drawer::MarkerBasicSetUp(this->spacecraft_body, 3);
  Drawer::MarkerBasicSetUp(this->spacecraft_velocity, 4);

  // Set the marker type.
  this->currentEllipsis.type = visualization_msgs::Marker::LINE_LIST;
  this->targetEllipsis.type = visualization_msgs::Marker::LINE_LIST;
  this->central_body.type = visualization_msgs::Marker::SPHERE;
  this->spacecraft_body.type = visualization_msgs::Marker::SPHERE;
  this->spacecraft_velocity.type = visualization_msgs::Marker::ARROW;

  Drawer::SetScales();
  Drawer::SetColors();

  //dimensions of ellipse
  float theta = 0;
  const float r_pe = 15150.0; //km
  const float r_apo= 56070.0; //km
  float a_orbit = (r_pe + r_apo)/2; //km
  float b_orbit = 0.7*a_orbit;

  // Set the initial pose of SC body
  this->spacecraft_body.pose.position.x = SCALE_FACTOR*b_orbit*sin(theta);
  this->spacecraft_body.pose.position.y = SCALE_FACTOR*(a_orbit*(1-cos(theta)) - r_pe);
  this->spacecraft_body.pose.position.z = 0;

  // Create the vertices for the lines
  for (uint32_t i = 0; i < 1000; ++i)
  {
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;

    this->currentEllipsis.points.push_back(p);
    this->targetEllipsis.points.push_back(p);
  }

  //create initial velocity arrow
  geometry_msgs::Point arrowBegin = spacecraft_body.pose.position;
  this->spacecraft_velocity.points.push_back(arrowBegin);
  this->spacecraft_velocity.points.push_back(arrowBegin);

  return;
}

#endif