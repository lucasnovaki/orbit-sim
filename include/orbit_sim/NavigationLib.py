import numpy as np
from orbit_sim.EnvironmentSim import Orbit2d, Solver2d
from orbit_sim.msg import Orbit2d as OrbitMsg
from orbit_sim.srv import ApplyThrust
import rospy
import math

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
        target.w_orbit = self.currentOrbit.w_orbit
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
        transferPsbl = self.checkTransferPsbl(self.targetOrbit, self.currentOrbit)
        if transferPsbl:
            if self.targetOrbit.a_orbit > self.currentOrbit.a_orbit:
                extOrbit = self.targetOrbit
                intOrbit = self.currentOrbit
                mod = 1
            else:
                intOrbit = self.targetOrbit
                extOrbit = self.currentOrbit
                mod = 2

            r_per_int = intOrbit.a_orbit*(1-intOrbit.e_orbit)
            v_per_int = np.sqrt(Solver2d.mi*(1+intOrbit.e_orbit)/(intOrbit.a_orbit*(1-intOrbit.e_orbit)))
            r_apo_ext = extOrbit.a_orbit*(1+extOrbit.e_orbit)
            v_apo_ext = np.sqrt(Solver2d.mi*(1-extOrbit.e_orbit)/(extOrbit.a_orbit*(1+extOrbit.e_orbit)))

            transferOrbit.a_orbit = (r_per_int + r_apo_ext)/2
            transferOrbit.e_orbit = 1 - r_per_int/transferOrbit.a_orbit
            transferOrbit.w_orbit = self.currentOrbit.w_orbit

            v_per_trs = np.sqrt(Solver2d.mi*(1+transferOrbit.e_orbit)/(transferOrbit.a_orbit*(1-transferOrbit.e_orbit)))
            v_apo_trs = np.sqrt(Solver2d.mi*(1-transferOrbit.e_orbit)/(transferOrbit.a_orbit*(1+transferOrbit.e_orbit)))

            print("V_per_int: {:.2f}".format(v_per_int))
            print("V_per_trs: {:.2f}".format(v_per_trs))
            print("V_apo_trs: {:.2f}".format(v_apo_trs))
            print("V_apo_ext: {:.2f}".format(v_apo_ext))

            if mod == 1:
                print('Transfer mode 1')
                dv_int = v_per_trs - v_per_int
                dv_ext = v_apo_ext - v_apo_trs
                maneuever_lst = [([0, dv_int], 0),([0, dv_ext], math.pi)]

            else:
                print('Transfer mode 2')
                dv_int = v_per_int - v_per_trs
                dv_ext = v_apo_trs - v_apo_ext
                maneuever_lst = [([0, dv_ext], math.pi), ([0, dv_int], 0)]

            print("Transfer orbit and maneuver list calculated")
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
        print(transfer.maneuever_lst)
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

