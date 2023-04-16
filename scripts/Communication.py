#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
import time

# Definition der globalen Variablen
message = ""
msg_rcvd = False

# Callback-Funktion zum Update der Variablen message
def callback1(data):
    global message
    global msg_rcvd
    global initTime

    currentTimestamp = time.time() - initTime
    message = data.data + " | Timestamp: {:.2f}".format(currentTimestamp)
    msg_rcvd = True


# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('communication', anonymous=True)
        
        # Start des Subscriber
        sub_comms1 = rospy.Subscriber("/states_true", String, callback1)
        sub_comms2 = rospy.Subscriber("/satellite_output", String, callback1)
        sub_comms3 = rospy.Subscriber("/states_trt", String, callback1)

        
        # Initialisierung des Publisher
        pub_comms = rospy.Publisher("/pretty_print", String, queue_size = 20)

        #Initialize time variable
        initTime = time.time()
        
        # Ausfuehrung der while-Schleife bis der Node gestoppt wird
        while not rospy.is_shutdown():

            # Check if message received
            if(msg_rcvd):
                pub_comms.publish(message)
                msg_rcvd = False

            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
