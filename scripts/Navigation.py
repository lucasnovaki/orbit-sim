#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from orbit_sim.srv import SetNewOrbit
from orbit_sim.msg import Orbits
from orbit_sim.NavigationLib import *
import numpy as np

### Global variables
NODE_RATE = 0.2

# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('navigation', anonymous=True)
        r = rospy.Rate(NODE_RATE)

        #Initialize Navigator and Planner instances 
        transferPlanner = Planner()
        spaceNavigator = Navigator2d(transferPlanner)

          
        # Initialisierung des Publisher / Server
        pub_nav_thrust = rospy.Publisher("/navigation/thrust", Point, queue_size = 1)
        sub_orbit_params = rospy.Subscriber("/simulation_data/orbit_params", Orbits, spaceNavigator.updateCurrentOrbit)

        #server_transfer = rospy.Service("/navigation/SetNewOrbit", SetNewOrbit, callbackTransferSrv)
        server_transfer = rospy.Service("/navigation/SetNewOrbit", SetNewOrbit, spaceNavigator.callbackSetTransfer)

        # Create a ROS Timer for sending data
        rospy.Timer(rospy.Duration(10.0/1000.0), transferPlanner.checkForManeuver)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
