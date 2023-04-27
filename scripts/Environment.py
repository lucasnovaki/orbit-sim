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
        
        # Initialisierung des Publisher
        pub_environment1 = rospy.Publisher("/simulation_data", String, queue_size = 20)
        
        solver = Solver()
        
        # Ausfuehrung der while-Schleife bis der Node gestoppt wird
        while not rospy.is_shutdown():
                
            if(continue_integration):
                solver.step()

            pub_environment1.publish("x = {:.6f}, y = {:.6f}, vx = {:.6f}, vy = {:.6f}".format(*solver.getStates()))

            time.sleep(0.5)

    except rospy.ROSInterruptException:
        pass
