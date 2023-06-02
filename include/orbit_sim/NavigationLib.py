import numpy as np
from orbit_sim.EnvironmentSim import Orbit2d
from orbit_sim.msg import Orbit2d as OrbitMsg
from orbit_sim.srv import ApplyThrust
import rospy

class Navigator2d(object):
    def __init__(self, planner):
        self.planner = planner
        self.currentOrbit = Orbit2d()
        self.pubTargetOrbit = rospy.Publisher("/navigation/target_orbit_params", OrbitMsg, queue_size = 1)

    def callbackSetTransfer(self, orbit_msg):

        #create new transfer instance
        target = Orbit2d()
        target.a_orbit = orbit_msg.a_orbit
        target.e_orbit = orbit_msg.e_orbit
        target.w_orbit = orbit_msg.w_orbit
        target.e_vector = target.calculateEccVec()
        self.transfer = self.createTransfer(target)
        print("Transfer created")

        #let planner handle it
        if (self.transfer): 
            self.pushToPlanner(self.transfer)

            #publish target to createvisual
            targetMsg = OrbitMsg()
            targetMsg.a_orbit = self.transfer.targetOrbit.a_orbit
            targetMsg.e_orbit = self.transfer.targetOrbit.e_orbit
            targetMsg.w_orbit = self.transfer.targetOrbit.w_orbit
            self.pubTargetOrbit.publish(targetMsg)

            return "Transfer planned."
        else:
            return "Error with transfer planning"


    def updateCurrentOrbit(self, current_orbit):
        #update true anomaly for planner 
        self.planner.currentTheta = current_orbit.theta_orbit

        #update current orbit for next transfers
        self.currentOrbit.a_orbit = current_orbit.a_orbit
        self.currentOrbit.e_orbit = current_orbit.e_orbit
        self.currentOrbit.w_orbit = current_orbit.w_orbit
        self.currentOrbit.e_vector = self.currentOrbit.calculateEccVec()

    def createTransfer(self, targetOrbit):
        if self.currentOrbit:
            transfer = Transfer2d(targetOrbit, self.currentOrbit)
            if (transfer.transferOrbit):
                return transfer
            else:
                print("Navigator cannot create transfer because target orbit intersects current orbit")
                return None
        else:
            print("Navigator cannot create transfer because it has no current orbit info")
            return None

    def pushToPlanner(self, transfer):
        self.planner.programTransfer(transfer)

class Transfer2d(object):
    def __init__(self, targetOrbit, currentOrbit):
        self.targetOrbit = targetOrbit
        self.currentOrbit = currentOrbit
        self.transferOrbit, self.maneuever_lst = self.calculateTransfer()

    def calculateTransfer(self):
        #dummy function for testing
        transferOrbit = Orbit2d()
        transferPsbl = self.checkForHohman(self.targetOrbit, self.currentOrbit)
        if (transferPsbl):
            maneuever_lst = [([0, 0], 0)]
            print("Transfer orbit and maneuver list calculated")
            return transferOrbit, maneuever_lst
        else:
            return None, []

    def checkForHohman(self, orbit1, orbit2):
        # verify that current and target orbit do not intersect

        #check high (extern) and low (intern) energy orbits
        if orbit1.a_orbit > orbit2.a_orbit:
            extOrbit = orbit1
            intOrbit = orbit2
        else:
            extOrbit = orbit2
            intOrbit = orbit1

        #find angle between periapsis lines
        beta = np.arccos(np.dot(extOrbit.e_vector[0], intOrbit.e_vector[0])/(extOrbit.e_orbit*intOrbit.e_orbit))
        
        #check if periapsis of inner orbit is larger than r(beta)
        r_inner = intOrbit.a_orbit*(1+intOrbit.e_orbit)
        r_outer = extOrbit.a_orbit*(1-extOrbit.e_orbit**2)/(1+extOrbit.e_orbit*np.cos(beta))

        if r_inner < r_outer: return True
        else: return False


class Planner(object):
    def __init__(self, tol = 0.05):
        self.queue = []
        self.currentTheta = None
        self.currentManeuever = None
        self.tol = tol # tolerance to detect maneuever point
        self.transfer_programmed = 0
        self.clientApplyThrust = rospy.ServiceProxy("/environment/ApplyThrust", ApplyThrust)

    def checkForManeuever(self, orbit_msg):
        #callback from ros timer
        if self.transfer_programmed:
            if abs(self.currentTheta - self.currentManeuever[1]) < self.tol:
                self.executeManeuever(self.currentManeuever[0]) 

    def programTransfer(self, transfer):
        self.queue.extend(transfer.maneuever_lst)
        if not self.transfer_programmed:
            self.currentManeuever = self.queue.pop(0)
            self.transfer_programmed = 1

    def executeManeuever(self, dv):
        #call thrust service
        dv_x = dv[0]
        dv_y = dv[1]
        self.clientApplyThrust(dv_x, dv_y)
        print('Thrust applied: dv = ({:.2f}, {:.2f})'.format(dv_x, dv_y))

        if len(self.queue) == 0:
            # if queue empty stop checking for theta
            self.transfer_programmed = 0
        else:
            #get next maneuver
            self.currentManeuever = self.queue.pop(0)

