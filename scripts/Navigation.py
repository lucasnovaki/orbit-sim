#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np

#Global variables
NODE_RATE = 0.2

# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('navigation', anonymous=True)
        r = rospy.Rate(NODE_RATE)
          
        # Initialisierung des Publisher
        pub_nav_thrust = rospy.Publisher("/navigation/thrust", Point, queue_size = 1)
        
        # Ausfuehrung der while-Schleife bis der Node gestoppt wird
        while not rospy.is_shutdown():              
                
            delta_v = Point()
            delta_v.x = 0.2*np.random.randn()
            delta_v.y = 0.2*np.random.randn()
            delta_v.z = 0

            pub_nav_thrust.publish(delta_v)

            r.sleep()

    except rospy.ROSInterruptException:
        pass
