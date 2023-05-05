import numpy as np
from geometry_msgs.msg import Vector3
from orbit_sim.msg import State2d
import rospy

class Solver2d(object):

    grav_const = 6.674e-20 # km^3/(kg s^2)
    earth_mass = 5.9722e24 # kg
    mi = grav_const*earth_mass # km^3/s^2
    r_pe = 15150.0 #km
    r_apo = 56070.0 #km
    a_orbit = (r_pe + r_apo)/2 #km
    v_pe = np.sqrt(mi*(2/r_pe - 1/a_orbit))

    def __init__(self, initState, dt):
        #solver variables
        self.dt = dt
        self.currentTime = 0
        self.initState = initState
        self.currentState = initState
    
    def step(self, event=None):

        #get derivative
        self.diff = self.calculateDerivative()

        #integration
        self.currentState = self.currentState + self.diff*self.dt

        #update time
        self.currentTime += self.dt

    def calculateDerivative(self):

        #so it is easier to manipulate
        x = self.currentState[0,0]
        y = self.currentState[0,1]
        dx = self.currentState[0,2]
        dy = self.currentState[0,3]

        #calculate rotation matrix from current state
        theta = np.arctan2(y, x)
        rotMatrix = self.getRotMatrix(theta)

        #derivative in radial/tangencial frame
        ddxTang = -np.array([[Solver2d.mi/(x**2 + y**2)], [0]])

        #get derivative in global frame
        diffGlobal = np.vstack(( np.array([[dx],[dy]]), np.matmul(rotMatrix, ddxTang) )).transpose()

        return diffGlobal

    def getRotMatrix(self, alpha):
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha),np.cos(alpha)]])


class Spacecraft2d(Solver2d):
    def __init__(self, id, initState = np.array([[0, -15150.0, 6.436, 0]]), dt = 1):

        #spacecraft id
        self.id = id

        #initialize solver and inherit methods
        super(Spacecraft2d, self).__init__(initState, dt)

        #publisher setup
        self.pubSimulationData = rospy.Publisher("/simulation_data/states", State2d, queue_size = 1)

    def applyThrust(delta_v):
        self.currentState[0,2] += delta_v[0, 1]
        self.currentState[0,3] += delta_v[0, 2]

    def getStates(self):
        return (self.currentState[0,0], self.currentState[0,1], self.currentState[0,2], self.currentState[0,3])

    def publish_data(self, event=None):
        pos_x, pos_y, vel_x, vel_y = self.getStates()

        state = State2d()
        state.position = Vector3(pos_x, pos_y, 0)
        state.velocity = Vector3(vel_x, vel_y, 0)

        self.pubSimulationData.publish(state)
        return

class Orbit2d(object):
    pass

class Maneuever2d(object):
    pass

