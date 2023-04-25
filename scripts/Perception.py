#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
import time
import numpy as np

# Definition der globalen Variablen
global std_x
global std_y
global std_z

# Callback-Funktion zum Update der Variablen message
def callback1(data):
    pass


# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('perception', anonymous=True)
        
        # Start des Subscriber
        sub_perception1 = rospy.Subscriber("/states_raw", String, callback1)
        
        # Initialisierung des Publisher
        pub_perception1 = rospy.Publisher("/states_trt", String, queue_size = 20)

        #Initialize time variable
        controlTime = 3 # seconds
        lastTime = time.time()

        #Initialize additional variables
        std_x = 1
        std_y = 0.5
        std_z  = 3

        # Ausfuehrung der while-Schleife bis der Node gestoppt wird
        while not rospy.is_shutdown():

            if time.time() - lastTime > controlTime:

                #Update last time
                lastTime = time.time()
                  
                #Get states
                x = std_x*np.random.randn()
                y = std_y*np.random.randn()
                z = std_z*np.random.randn()

                # Publishen der Nachricht
                pub_perception1.publish('Perception message: state = ({:.2f},{:.2f},{:.2f})'.format(x, y, z))


            time.sleep(0.3)

    except rospy.ROSInterruptException:
        pass
