#!/usr/bin/env python

# Einbindung der notwendigen Bibliothken
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from orbit_sim.srv import ApplyThrust
from orbit_sim.EnvironmentSim import *
import numpy as np

#Global variable
global solver 

# Callback to control Solver 
def callbackNavigation(delta_vel):
    global solver
    if len(solver.lst_ids) > 0:
        id = solver.lst_ids[0]
        solver.spacecrafts[id].applyThrust(np.array([[delta_vel.x, delta_vel.y]]) )
    return

def callbackThrustSrv(args):
    global solver
    if args.id in solver.spacecrafts.keys():
        solver.spacecrafts[args.id].applyThrust(np.array([[args.x, args.y]]) )
        return "Thrust: delta vel = ({:.4f}, {:.4f})".format(args.x, args.y)
    else:
        return "Thrust error. Spacecraft {:d} non-existent".format(args.id)
    
# Hauptprogramm
if __name__ == '__main__':

    try:
        # Initialisierung und Start des Nodes
        rospy.init_node('environment', anonymous=True)

        #Initialize environment solver
        solver = Solver2d(dt = 1)
        
        #solver.add_spacecraft(id, np.array([initStateParam]))

        #Callback for Navigation node/Thrust service
        sub_nav_thrust = rospy.Subscriber("/navigation/thrust", Point, callbackNavigation)
        server_thrust = rospy.Service("/environment/ApplyThrust", ApplyThrust, callbackThrustSrv)

        # Create a ROS Timer for numerical simulation update equation
        rospy.Timer(rospy.Duration(5.0/1000.0), solver.step)

        # Create a ROS Timer for sending data
        rospy.Timer(rospy.Duration(25.0/1000.0), solver.publish_states)

        # Create a ROS Timer for updating ellipsis shape
        rospy.Timer(rospy.Duration(100.0/1000.0), solver.publish_orbit_params)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
