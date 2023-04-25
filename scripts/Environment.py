#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
from orbit_sim.OrbitSolver import *
import time

# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('environment', anonymous=True)
        
        # Initialisierung des Publisher
        pub_environment1 = rospy.Publisher("/simulation_data", String, queue_size = 20)

        
        solver = Solver("this content")
        
        # Ausfuehrung der while-Schleife bis der Node gestoppt wird
        while not rospy.is_shutdown():
                
            # Publishen der Nachricht
            pub_environment1.publish(solver.getContent())

            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
