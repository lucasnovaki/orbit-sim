#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np

### Global variables
NODE_RATE = 0.2

### Functions

def applyRandomThrust(pub, std_dev):
    #create delta v vector
    delta_v = Point()
    delta_v.x = std_dev*np.random.randn()
    delta_v.y = std_dev*np.random.randn()
    delta_v.z = 0

    #publish 
    pub.publish(delta_v)


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
                
            applyRandomThrust(pub_nav_thrust, 0.2)

            r.sleep()

    except rospy.ROSInterruptException:
        pass
