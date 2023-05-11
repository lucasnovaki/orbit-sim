#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from orbit_sim.srv import SetNewOrbit
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

def callbackTransferSrv(target_orbit):

    #init subscriber and get orbit
    current_orbit = None

    # calculate transfer orbit
    getControlOutput(target_orbit, current_orbit)

    # instruction needs to be sent to 'scheduler'

    #return message
    return "[INFO] Target orbit: a = {:.0f}, e = {:.3f}".format(target_orbit.a_orbit, target_orbit.e_orbit)

def getControlOutput(orbit1, orbit2):
    #move to Navigator2d class (to-do)
    pass

# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('navigation', anonymous=True)
        r = rospy.Rate(NODE_RATE)
          
        # Initialisierung des Publisher / Server
        pub_nav_thrust = rospy.Publisher("/navigation/thrust", Point, queue_size = 1)
        server_transfer = rospy.Service("/navigation/SetNewOrbit", SetNewOrbit, callbackTransferSrv)

        # Ausfuehrung der while-Schleife bis der Node gestoppt wird
        while not rospy.is_shutdown():              
                
            applyRandomThrust(pub_nav_thrust, 0.2)

            r.sleep()

    except rospy.ROSInterruptException:
        pass
