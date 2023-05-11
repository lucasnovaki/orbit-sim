import numpy as np
from orbit_sim.EnvironmentSim import Orbit2d
from orbit_sim.msg import Orbit2d as OrbitMsg
import rospy

class Navigator2d(object):
    def __init__(self):
        pass

    def createTransfer(self):


    def pushToPlanner(self, planner, transfer):
        pass

class Transfer2d(object):
    def __init__(self, targetOrbit, currentOrbit):
        self.targetOrbit = targetOrbit
        self.currentOrbit = currentOrbit
        self.transferOrbit, self.firstManeuever, self.secondManeuever = self.calculateTransfer()

    def calculateTransfer(self):
        #to-do: fun description
        transferOrbit = Orbit2d()
        return None, None, None

class Planner(object):
    def __init__(self):
        pass