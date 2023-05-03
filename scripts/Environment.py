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
        #rate = rospy.Rate(100)
        
        solver = Solver()

        # Create a ROS Timer for numerical simulation
        rospy.Timer(rospy.Duration(1.0/1000.0), solver.step)

        # Create a ROS Timer for sending data
        rospy.Timer(rospy.Duration(10.0/100.0), solver.publish_data)
        
        rospy.spin()
                
        #   if(continue_integration):
        #       solver.step()

        #pub_environment1.publish("x = {:.6f}, y = {:.6f}, vx = {:.6f}, vy = {:.6f}".format(*solver.getStates()))
        #rate.sleep()

    except rospy.ROSInterruptException:
        pass
