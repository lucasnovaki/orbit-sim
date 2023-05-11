#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from orbit_sim.srv import ApplyThrust
from orbit_sim.EnvironmentSim import *
import numpy as np

#Global variable
global sc #spacecraft

# Callback to control Solver 
def callbackNavigation(delta_vel):
    global sc
    sc.applyThrust(np.array([[delta_vel.x, delta_vel.y]]) )
    return

def callbackThrustSrv(delta_vel):
    global sc
    sc.applyThrust(np.array([[delta_vel.x, delta_vel.y]]) )
    return "Thrust: delta vel = ({:.4f}, {:.4f})".format(delta_vel.x, delta_vel.y)
    
# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('environment', anonymous=True)
        
        #get initial conditions for simulation
        initStateParam = rospy.get_param('/initialState')

        #create spacecraft instance
        sc = Spacecraft2d(0, initState = np.array([initStateParam]), dt = 2.5)

        #Callback for Navigation node/Thrust service
        sub_nav_thrust = rospy.Subscriber("/navigation/thrust", Point, callbackNavigation)
        server_thrust = rospy.Service("/environment/ApplyThrust", ApplyThrust, callbackThrustSrv)

        # Create a ROS Timer for numerical simulation update equation
        rospy.Timer(rospy.Duration(5.0/1000.0), sc.step)

        # Create a ROS Timer for sending data
        rospy.Timer(rospy.Duration(25.0/1000.0), sc.publish_states)

        # Create a ROS Timer for updating ellipsis shape
        rospy.Timer(rospy.Duration(100.0/1000.0), sc.publish_orbit_params)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
