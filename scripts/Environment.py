#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
from orbit_sim.OrbitSolver import *
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

        #create instance for solver
        solver = Solver(initState = np.array([initStateParam]))

        # Create a ROS Timer for numerical simulation update equation
        rospy.Timer(rospy.Duration(1.0/1000.0), solver.step)

        # Create a ROS Timer for sending data
        rospy.Timer(rospy.Duration(10.0/1000.0), solver.publish_data)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
