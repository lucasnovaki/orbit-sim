#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
import time
import numpy as np
import random
from turtlesim.srv import TeleportAbsolute

# Definition der globalen Variablen
message = "Not Caught!"
move_turtle = False

# Callback-Funktion zum Update der Variablen message
def callback1(data):
    global move_turtle
    global message

    if message == data.data:
        move_turtle = False
    else:
        move_turtle = True


# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('snake', anonymous=True)
        
        # Start des Subscriber
        sub_turtle1 = rospy.Subscriber("/caught", String, callback1)
        
        # Initialisierung des Publisher
        pub_turtle1 = rospy.Publisher("/talking_turtle", String, queue_size = 20)
        
        # Ausfuehrung der while-Schleife bis der Node gestoppt wird
        while not rospy.is_shutdown():

            # Ueberpruefung ob die Schildkroete teleoperiert werden muss
            # hier fehlt noch etwas
            if(move_turtle):

                # Zuweisung einer Zufallsposition an welche die Schildkroete teleoperiert wird
                # muss zu gegebener Zeit auskommentiert werden
                i = random.randint(1, 10)
                j = random.randint(1, 10)
                k = random.uniform(0, 2*np.pi)

                # Aufruf des Service zum Teleoperieren der Schildkroete
                # hier fehlt noch etwas
                try:
                    teleport = rospy.ServiceProxy('/turtle2/teleport_absolute', TeleportAbsolute)
                    resp = teleport(i,j,k)
                    print ('Teleported!')
                except rospy.ServiceException, e:
                    print 'Service call failed'
                  
                
                # Publishen der Nachricht, dass die Schildkroete gefangen wurde
                # hier fehlt noch etwas
                pub_turtle1.publish('Turtle caught')

            else:
                pub_turtle1.publish('Turtle not caught')

            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
