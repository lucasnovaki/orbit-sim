#ifndef CREATE_VISUAL_H
#define CREATE_VISUAL_H

#include <ros/ros.h>
#include <cmath>
#include <map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <orbit_sim/State2d.h>
#include <orbit_sim/Orbit2d.h>
#include <orbit_sim/Orbits.h>
#include <orbit_sim/BodyID.h>
#include <geometry_msgs/Vector3.h>
#include <ros/console.h>

#define SCALE_FACTOR 0.001
#define ELLIPSIS_LINE_SIZE 1000.0f

using namespace std;

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
        ros::Subscriber spawn_add_sub;
        ros::Subscriber spawn_del_sub;
        ros::Subscriber ellipsis_sub;
        ros::Subscriber tar_ellipsis_sub;

        //Markers
        visualization_msgs::Marker central_body;
        map <uint16_t, visualization_msgs::Marker> spacecraft_body_map;
        map <uint16_t, visualization_msgs::Marker> spacecraft_velocity_map;
        map <uint16_t, visualization_msgs::Marker> currentEllipsis_map;
        map <uint16_t, visualization_msgs::Marker> targetEllipsis_map;

        uint16_t markerTotal = 0;

        // Main publisher method
        void PublishMarkers();

        //Callback methods
        void updateVelocityArrow(uint16_t, geometry_msgs::Vector3, geometry_msgs::Vector3);
        void updateSpacecraftBody(uint16_t, geometry_msgs::Vector3);
        void PosCallback(const orbit_sim::State2d::ConstPtr&);
        void CurrentOrbitCallback(const orbit_sim::Orbits::ConstPtr&);
        void TargetOrbitCallback(const orbit_sim::Orbits::ConstPtr&);
        void callbackSpawnerAdd(const orbit_sim::State2d::ConstPtr&);
        void callbackSpawnerDelete(const orbit_sim::BodyID::ConstPtr&);

        //Auxiliary methods
        void drawEllipsis(const orbit_sim::Orbit2d, visualization_msgs::Marker&);
        void PublishArray(ros::Publisher, map <uint16_t, visualization_msgs::Marker>);

        //Setup methods
        void MarkerSetUp();
        void MarkerBasicSetUp(visualization_msgs::Marker&, int, uint8_t);
        void CommsSetUp(ros::NodeHandle&);
        void SetColors();
        void SetScales();

        void SetUp(ros::NodeHandle& n){
            this->CommsSetUp(n);
            this->MarkerSetUp();
            ROS_DEBUG("createvisual set up finished");
        }
 };

 void Drawer::callbackSpawnerAdd(const orbit_sim::State2d::ConstPtr& state){
    //add news markers to maps

    ROS_INFO("Callback triggered");

    //Define new markers
    visualization_msgs::Marker newMarkerPos;
    visualization_msgs::Marker newMarkerVel;
    visualization_msgs::Marker newMarkerCurElps;

    //SetUp Functions
    Drawer::MarkerBasicSetUp(newMarkerPos, state->id[0], visualization_msgs::Marker::SPHERE);
    Drawer::MarkerBasicSetUp(newMarkerVel, state->id[0], visualization_msgs::Marker::ARROW);
    Drawer::MarkerBasicSetUp(newMarkerCurElps, state->id[0], visualization_msgs::Marker::LINE_LIST);

    //Assign new map entry to created markers
    this->spacecraft_body_map[state->id[0]] = newMarkerPos;
    ROS_INFO("Spacecraft body %d added", state->id[0]);
    this->spacecraft_velocity_map[state->id[0]] = newMarkerVel;
    ROS_INFO("Velocity arrow %d added", state->id[0]);
    this->currentEllipsis_map[state->id[0]] = newMarkerCurElps;
    ROS_INFO("Ellipsis %d added", state->id[0]);

    //Update control variable
    this->markerTotal++;
 }

  void Drawer::callbackSpawnerDelete(const orbit_sim::BodyID::ConstPtr& body){

    //Set action to delete
    auto itMapVel = this->spacecraft_velocity_map.find(body->id);
    if (itMapVel!= this->spacecraft_velocity_map.end()){
        itMapVel->second.action = visualization_msgs::Marker::DELETE;
    }

    auto itMapPos = this->spacecraft_body_map.find(body->id);
    if (itMapPos!= this->spacecraft_body_map.end()){
        itMapPos->second.action = visualization_msgs::Marker::DELETE;
    }

    auto itMapElps = this->currentEllipsis_map.find(body->id);
    if (itMapElps!= this->currentEllipsis_map.end()){
        itMapElps->second.action = visualization_msgs::Marker::DELETE;
    }

    auto itMapTarElps = this->targetEllipsis_map.find(body->id);
    if (itMapTarElps!= this->targetEllipsis_map.end()){
        itMapTarElps->second.action = visualization_msgs::Marker::DELETE;
    }

    //Delete markers in rviz
    Drawer::PublishMarkers();
    Drawer::PublishArray(this->tar_ellipsis_pub, targetEllipsis_map); //new function


    //Delete markers saved in map
    this->spacecraft_body_map.erase(body->id);
    this->spacecraft_velocity_map.erase(body->id);
    this->currentEllipsis_map.erase(body->id);
    this->targetEllipsis_map.erase(body->id);

    //Update control variable
    this->markerTotal--;
 }

 void Drawer::updateVelocityArrow(uint16_t id, geometry_msgs::Vector3 pos, geometry_msgs::Vector3 vel){
    geometry_msgs::Point arrowBegin;
    geometry_msgs::Point arrowEnd;

    arrowBegin.x = SCALE_FACTOR*pos.x;
    arrowBegin.y = SCALE_FACTOR*pos.y;
    arrowBegin.z = 0;
    arrowEnd.x = vel.x + arrowBegin.x;
    arrowEnd.y = vel.y + arrowBegin.y;
    arrowEnd.z = 0;

    auto itMapVel = this->spacecraft_velocity_map.find(id);
    if (itMapVel!= this->spacecraft_velocity_map.end()){
        itMapVel->second.points[0] = arrowBegin;
        itMapVel->second.points[1] = arrowEnd;
    }
    
    return;
}

void Drawer::updateSpacecraftBody(uint16_t id, geometry_msgs::Vector3 pos){
  auto itMapPos = this->spacecraft_body_map.find(id);
  if (itMapPos!= this->spacecraft_body_map.end()){
    itMapPos->second.pose.position.x = SCALE_FACTOR*pos.x;
    itMapPos->second.pose.position.y = SCALE_FACTOR*pos.y;
  }
  return;
}

void Drawer::PosCallback(const orbit_sim::State2d::ConstPtr& state){

    for(uint16_t i = 0; i < state->id.size(); ++i){
        uint16_t currentId = state->id[i];
        Drawer::updateSpacecraftBody(currentId, state->position[i]);
        Drawer::updateVelocityArrow(currentId, state->position[i], state->velocity[i]);
    }

    return;
}


void Drawer::drawEllipsis(const orbit_sim::Orbit2d orbitParams, visualization_msgs::Marker& ellipsisObj){
    // Get update values for orbit params
    float current_a_orbit = orbitParams.a_orbit;
    float current_e_orbit = orbitParams.e_orbit;
    float current_b_orbit = current_a_orbit*sqrt(1 - pow(current_e_orbit,2));

    if (current_e_orbit <= 1 & current_e_orbit > 0){
      
      //calculate orbit shape only if its indeed ellipsis
      float r_pe = current_a_orbit*(1 - current_e_orbit);
        
      // Create the vertices for the lines
      for (uint32_t i = 0; i < ELLIPSIS_LINE_SIZE; ++i)
      {
        float x_i = current_b_orbit * cos(i / ELLIPSIS_LINE_SIZE * 2 * M_PI);
        float y_i = current_a_orbit * (sin(i / ELLIPSIS_LINE_SIZE * 2 * M_PI) + 1) - r_pe;

        geometry_msgs::Point p;
        p.x = SCALE_FACTOR*x_i;
        p.y = SCALE_FACTOR*y_i;
        p.z = 0;

        ellipsisObj.points[i] = rotate2dPoint(p, -orbitParams.w_orbit + M_PI/2);
      }
    }

    return;
}

void Drawer::CurrentOrbitCallback(const orbit_sim::Orbits::ConstPtr& orbitsMsg){
  
    for(uint16_t i = 0; i < orbitsMsg->id.size(); ++i){
        auto itMapElps = this->currentEllipsis_map.find(orbitsMsg->id[i]);
        if(itMapElps != this->currentEllipsis_map.end()){
            Drawer::drawEllipsis(orbitsMsg->orbit[i], itMapElps->second);
        }
        
    }
    
}

void Drawer::TargetOrbitCallback(const orbit_sim::Orbits::ConstPtr& orbitMsg){
    //get ID
    int16_t currentId = orbitMsg->id[0];

    //Initialize and set-up marker
    visualization_msgs::Marker newMarkerTarElps;
    Drawer::MarkerBasicSetUp(newMarkerTarElps, currentId, visualization_msgs::Marker::LINE_LIST);
    
    //Update entry or create new if non existent
    this->targetEllipsis_map[currentId] = newMarkerTarElps;
    ROS_INFO("Target ellipsis %d added/altered", currentId);

    //Update points
    Drawer::drawEllipsis(orbitMsg->orbit[0], this->targetEllipsis_map[currentId]);

    //Define marker array for publishing
    visualization_msgs::MarkerArray targetEllipsis;

    //Fill array of target ellipsis with old and new target markers
    for(auto it = this->targetEllipsis_map.begin(); it != this->targetEllipsis_map.end(); ++it){
        targetEllipsis.markers.push_back(it->second);
    }
    
    //Publish to rviz
    this->tar_ellipsis_pub.publish(targetEllipsis);
    return;
}

void Drawer::PublishArray(ros::Publisher pub, map <uint16_t, visualization_msgs::Marker> mmap){
    if (mmap.size()>0){

        //Define message datatype
        visualization_msgs::MarkerArray arraySc;

        //Fill array
        for(auto it = mmap.begin(); it!=mmap.end(); ++it){
            arraySc.markers.push_back(it->second);
        }

        //Publish
        pub.publish(arraySc);
    }
    return;
}


void Drawer::PublishMarkers(){

    int sizeMap = this->spacecraft_body_map.size();

    if (sizeMap>0){
        visualization_msgs::MarkerArray spacecraft_body;
        visualization_msgs::MarkerArray spacecraft_velocity;
        visualization_msgs::MarkerArray currentEllipsis;

        //Fill array of SC bodies
        for(auto it = spacecraft_body_map.begin(); it!=spacecraft_body_map.end(); ++it){
            spacecraft_body.markers.push_back(it->second);
        }
        ROS_DEBUG("Spacecrafts added to marker array");

        //Fill array of SC velocities
        for(auto it = spacecraft_velocity_map.begin(); it!=spacecraft_velocity_map.end(); ++it){
            spacecraft_velocity.markers.push_back(it->second);
        }
        ROS_DEBUG("Velocitites added to marker array");

        //Fill array of SC trajectories
        for(auto it = currentEllipsis_map.begin(); it != currentEllipsis_map.end(); ++it){
            currentEllipsis.markers.push_back(it->second);
        }
        ROS_DEBUG("Trajectories added to marker array");


        //Finally publish arrays to rviz
        this->spacecraft_body_pub.publish(spacecraft_body);
        this->spacecraft_vel_pub.publish(spacecraft_velocity);
        this->ellipsis_pub.publish(currentEllipsis);
        
    }   
    
    // Publish central body (if it is constant there is no need though)
    this->central_body_pub.publish(this->central_body);
    
    return;
}

void Drawer::CommsSetUp(ros::NodeHandle& n){
    //Publishers for rviz
    this->ellipsis_pub = n.advertise<visualization_msgs::MarkerArray>("spacecraft/ellipsis", 1);
    this->tar_ellipsis_pub = n.advertise<visualization_msgs::MarkerArray>("spacecraft/target_ellipsis", 1);
    this->central_body_pub = n.advertise<visualization_msgs::Marker>("spacecraft/central_body", 1);
    this->spacecraft_body_pub = n.advertise<visualization_msgs::MarkerArray>("spacecraft/spacecraft_body", 1);
    this->spacecraft_vel_pub = n.advertise<visualization_msgs::MarkerArray>("spacecraft/spacecraft_velocity", 1);
    
    //Subscribers
    this->spawn_add_sub = n.subscribe("/SpawnControl/spawn", 1, &Drawer::callbackSpawnerAdd, this);
    this->spawn_del_sub = n.subscribe("/SpawnControl/delete", 1, &Drawer::callbackSpawnerDelete, this);
    this->spacecraft_pos_sub = n.subscribe("/simulation_data/states", 1, &Drawer::PosCallback, this);
    this->ellipsis_sub = n.subscribe("/simulation_data/orbit_params", 1, &Drawer::CurrentOrbitCallback, this);
    this->tar_ellipsis_sub = n.subscribe("/navigation/target_orbit_params", 1, &Drawer::TargetOrbitCallback, this);

    return;
}

void Drawer::MarkerBasicSetUp(visualization_msgs::Marker& marker, int id, uint8_t markerType){

    //General set up applyable to all markers
    marker.header.frame_id = "visual_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "orbit";
    marker.id = id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = markerType;
    marker.lifetime = ros::Duration();
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    //Define neutral initial point
    geometry_msgs::Point pInit;
    pInit.x = 0;
    pInit.y = 0;
    pInit.z = 0;
    
    if (markerType == visualization_msgs::Marker::SPHERE){
        //Set color and scale
        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 2;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
    }
    else if (markerType == visualization_msgs::Marker::ARROW){
        //Set color and scale
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.4;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        //Init points with zero value
        marker.points.push_back(pInit);
        marker.points.push_back(pInit);
    }
    else if (markerType == visualization_msgs::Marker::LINE_LIST){
        //Set color and scale
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.scale.x = 0.1;

        //Initialize points with zero value
        for (uint32_t i = 0; i < ELLIPSIS_LINE_SIZE; ++i){
            marker.points.push_back(pInit);
        }
    }
    return;
}

void Drawer::SetColors(){
    // Set the color -- be sure to set alpha to something non-zero!

    this->central_body.color.r = 1.0f;
    this->central_body.color.g = 0.0f;
    this->central_body.color.b = 0.0f;
    this->central_body.color.a = 1.0;
    
}

void Drawer::SetScales(){
    // Set the scale of the marker
    //this->targetEllipsis.scale.x = 0.1;

    this->central_body.scale.x = SCALE_FACTOR*13000;
    this->central_body.scale.y = SCALE_FACTOR*13000;
    this->central_body.scale.z = SCALE_FACTOR*13000;
    
}

void Drawer::MarkerSetUp(){
  // Basic Setup
  
  Drawer::MarkerBasicSetUp(this->central_body, 99, visualization_msgs::Marker::SPHERE);
  Drawer::SetScales();
  Drawer::SetColors();

  return;
}

#endif