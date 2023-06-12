#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from orbit_sim.srv import Spawn, Delete
from orbit_sim.msg import State2d, BodyID
from geometry_msgs import Vector3

#Global variables
global lst_spacecraft


class Spawner:
    def __init__(self):
        
        #Start empty list for spacecrafts
        self.lst_spacecraft = []

        # Start des Subscriber
        self.srv_spawn = rospy.Service("/spawncontrol/Spawn", int, self.callbackSpawn)
        self.srv_del = rospy.Service("/spawncontrol/Delete", int, self.callbackDelete)

        # Initialisierung des Publisher
        self.pub_spawn = rospy.Publisher("/SpawnControl/spawn", int, queue_size = 1)
        self.pub_del = rospy.Publisher("/SpawnControl/delete", int, queue_size = 1)
        
    #Callback functions

    def callbackSpawn(self, spawn_msg):
        
        if spawn_msg.id in self.lst_spacecraft:
            return "[ERROR] Spacecraft ID exists already"
        else:
            self.lst_spacecraft.append(spawn_msg.id)
            pub_msg = State2d()
            pub_msg.id = [spawn_msg.id]
            pub_msg.position = [Vector3(spawn_msg.positionX , spawn_msg.positionY, 0)]
            pub_msg.velocity = [Vector3(spawn_msg.velocityX , spawn_msg.velocityY, 0)]
            self.pub_spawn.publish(pub_msg)
            return "[INFO] Spacecraft {:d} created".format(spawn_msg.id)


    def callbackDelete(self, del_msg):

        if del_msg.id not in self.lst_spacecraft:
            return "[ERROR] Spacecraft ID does not exist"
        else:
            self.lst_spacecraft.remove(del_msg.id)
            self.pub_del.publish(BodyID(del_msg.id))
            return "[INFO] Spacecraft {:d} removed".format(del_msg.id)

# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('spawncontrol', anonymous=True)

        #Init spawner
        spawner = Spawner()

        #get initial conditions and add first sc 
        initStateParam = rospy.get_param('/initialState')

        pub_msg = State2d()
        pub_msg.id = [1] #id
        pub_msg.position = [Vector3(initStateParam[0] , initStateParam[1], 0)] #position
        pub_msg.velocity = [Vector3(initStateParam[2] , initStateParam[3], 0)] #velocity
        spawner.pub_spawn.publish(pub_msg)

        # Wait for callbacks
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
