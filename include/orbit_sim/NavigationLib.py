import numpy as np
from orbit_sim.EnvironmentSim import Orbit2d, Solver2d
from geometry_msgs.msg import Point
from orbit_sim.msg import Orbit2d as OrbitMsg
from orbit_sim.srv import ApplyThrust
import rospy
import math

class Navigator2d(object):
    def __init__(self, planner):
        self.planner = planner
        self.currentOrbit = dict()
        self.pubTargetOrbit = rospy.Publisher("/navigation/target_orbit_params", OrbitMsg, queue_size = 1)

    def callbackSetTransfer(self, srv_msg):

        #create new transfer instance
        srv_msg.w_orbit = self.currentOrbit[srv_msg.id].w_orbit #apply restriction w_1 = w_2
        target = Orbit2d(orbitMsg= srv_msg)
        self.transfer = self.createTransfer(target)
        rospy.loginfo("Transfer created")

        #let planner handle it
        if (self.transfer): 
            self.pushToPlanner(srv_msg.id, self.transfer)

            #publish target to createvisual
            targetMsg = self.transfer.targetOrbit.toOrbitMsg()
            self.pubTargetOrbit.publish(targetMsg)
            return "Transfer planned."
        
        else:
            return "Error with transfer planning"


    def updateCurrentOrbit(self, Orbits):
    
        for id, orbit in zip(Orbits.id, Orbits.orbit):
            
            if id not in self.currentOrbit.keys():
                #create new dict entry if spacecraft is new
                self.currentOrbit[id] = Orbit2d(orbitMsg = orbit)
            else:
                #update current orbit for next transfers
                self.currentOrbit[id].setOrbit(orbit)

            #update true anomaly for planner 
            self.planner.currentTheta[id] = orbit.theta

            

    def createTransfer(self, id, targetOrbit):
        if self.currentOrbit[id]:
            transfer = Transfer2d(targetOrbit, self.currentOrbit[id])
            if (transfer.transferOrbit):
                return transfer
            else:
                rospy.logwarn("Navigator cannot create transfer because target orbit intersects current orbit")
                return None
        else:
            rospy.logerr("Navigator cannot create transfer because it has no current orbit info")
            return None

    def pushToPlanner(self, id, transfer):
        self.planner.programTransfer(id, transfer)

class Transfer2d(object):
    def __init__(self, targetOrbit, currentOrbit):
        self.targetOrbit = targetOrbit
        self.currentOrbit = currentOrbit
        self.transferOrbit, self.maneuever_lst = self.calculateTransfer()

    def calculateTransfer(self):
        # outputs maneuver list: [m1, m2]
        # maneuver (tuple) m1 = ([vel_x, vel_y], theta)
        # vel_x, vel_y defined in spacecraft reference frame (y == tangencial)
        # theta defined as true anomaly of orbit before applying thrust

        transferOrbit = Orbit2d()

        #check intersection restriction
        transferPsbl = self.checkTransferPsbl(self.targetOrbit, self.currentOrbit)
        
        if transferPsbl:

            #find out if target is lower or higher
            if self.targetOrbit.a_orbit > self.currentOrbit.a_orbit:
                extOrbit = self.targetOrbit
                intOrbit = self.currentOrbit
                mod = 1
            else:
                intOrbit = self.targetOrbit
                extOrbit = self.currentOrbit
                mod = 2

            #hohmann transfer: calculate velocities before and after thrust in periapsis/apoapsis
            r_per_int = intOrbit.a_orbit*(1-intOrbit.e_orbit)
            v_per_int = np.sqrt(Solver2d.mi*(1+intOrbit.e_orbit)/(intOrbit.a_orbit*(1-intOrbit.e_orbit)))
            r_apo_ext = extOrbit.a_orbit*(1+extOrbit.e_orbit)
            v_apo_ext = np.sqrt(Solver2d.mi*(1-extOrbit.e_orbit)/(extOrbit.a_orbit*(1+extOrbit.e_orbit)))

            transferOrbit.a_orbit = (r_per_int + r_apo_ext)/2
            transferOrbit.e_orbit = 1 - r_per_int/transferOrbit.a_orbit
            transferOrbit.w_orbit = self.currentOrbit.w_orbit

            v_per_trs = np.sqrt(Solver2d.mi*(1+transferOrbit.e_orbit)/(transferOrbit.a_orbit*(1-transferOrbit.e_orbit)))
            v_apo_trs = np.sqrt(Solver2d.mi*(1-transferOrbit.e_orbit)/(transferOrbit.a_orbit*(1+transferOrbit.e_orbit)))

            #log velocity in maneuver points
            rospy.loginfo("V_per_int: {:.2f}".format(v_per_int))
            rospy.loginfo("V_per_trs: {:.2f}".format(v_per_trs))
            rospy.loginfo("V_apo_trs: {:.2f}".format(v_apo_trs))
            rospy.loginfo("V_apo_ext: {:.2f}".format(v_apo_ext))

            #create list of maneuvers
            if mod == 1:
                rospy.loginfo('Transfer mode 1')
                dv_int = v_per_trs - v_per_int
                dv_ext = v_apo_ext - v_apo_trs
                maneuever_lst = [([0, dv_int], 0),([0, dv_ext], math.pi)]

            else:
                rospy.loginfo('Transfer mode 2')
                dv_int = v_per_int - v_per_trs
                dv_ext = v_apo_trs - v_apo_ext
                maneuever_lst = [([0, dv_ext], math.pi), ([0, dv_int], 0)]

            rospy.loginfo("Transfer orbit and maneuver list calculated")
            return transferOrbit, maneuever_lst
        else:
            return None, []

    def checkTransferPsbl(self, orbit1, orbit2):
        # verify that current and target orbit do not intersect
        a_1 = orbit1.a_orbit
        e_1 = orbit1.e_orbit
        a_2 = orbit2.a_orbit
        e_2 = orbit2.e_orbit
        C = (a_2*(1-e_2**2) - a_1*(1-e_1**2))/(a_1*e_2*(1-e_1**2) - a_2*e_1*(1-e_2**2))
        if abs(C) <= 1: return False
        else: return True


class Planner(object):
    def __init__(self, tol = 0.025):
        self.queues = dict()
        self.currentTheta = dict()
        self.tol = tol # tolerance to detect maneuever point [rad]
        self.transfer_programmed = 0

        ## communication
        self.clientApplyThrust = rospy.ServiceProxy("/environment/ApplyThrust", ApplyThrust)
        self.pubApplyThrust = rospy.Publisher("/navigation/thrust", Point, queue_size = 1)

    def checkForManeuever(self, orbit_msg):
        #callback from ros timer to verify point of maneuver
        for id in self.queues.keys():
            currentManeuver = self.queues[id][0]
            C1 = (abs(self.currentTheta[id] - currentManeuver) < self.tol)
            C2 = (abs(self.currentTheta[id] + 2*math.pi - currentManeuver) < self.tol)
            C3 = (abs(self.currentTheta[id] - 2*math.pi - currentManeuver) < self.tol)
            if C1 or C2 or C3:
                self.executeManeuever(id, currentManeuver) 

    def programTransfer(self, id, transfer):
        
        #start queue if sc is new
        if id not in self.queues.keys(): self.queues[id] = []

        # place planned kick burn at the queue end
        self.queues[id].extend(transfer.maneuever_lst)

        rospy.loginfo("Current maneuver for queue {:d}: {}".format(id, str(transfer.maneuever_lst)))


    def executeManeuever(self, id, dv):
        #call thrust service/ publish thrust msg
        self.clientApplyThrust(id, dv[0], dv[1])
        rospy.loginfo('Thrust published: dv = ({:.2f}, {:.2f})'.format(dv[0], dv[1]))

        self.queues[id].pop(0)

        if len(self.queues[id]) == 0:
            # if list empty stop checking for this spacecraft
            self.queues[id].pop(id)

