#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
from orbit_sim.EnvironmentSim import *
import time

#Global variables
continue_integration = True
continue_msg = "Y"
stop_msg = "N"

# Callback to control Solver 
def callback1(data):
    global continue_integration
    global continue_msg
    global stop_msg

    if continue_msg == data.data:
        continue_integration = True
    elif stop_msg == data.data:
        continue_integration = False

# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('environment', anonymous=True)
        
        #get initial conditions for simulation
        initStateParam = rospy.get_param('/initialState')

        #create spacecraft instance
        sc = Spacecraft2d(0, initState = np.array([initStateParam]), dt = 1)

        # Create a ROS Timer for numerical simulation update equation
        rospy.Timer(rospy.Duration(1.0/1000.0), sc.step)

        # Create a ROS Timer for sending data
        rospy.Timer(rospy.Duration(10.0/1000.0), sc.publish_states)

        # Create a ROS Timer for updating ellipsis shape
        rospy.Timer(rospy.Duration(100.0/1000.0), sc.publish_orbit_params)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
