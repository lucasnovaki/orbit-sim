import numpy as np
from geometry_msgs.msg import Vector3
from orbit_sim.msg import State2d
from orbit_sim.msg import Orbit2d as OrbitMsg
import rospy
import math

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

        #orbit shape
        self.orbit = Orbit2d(self)

        #publisher setup for 2d states
        self.pubSimulationData = rospy.Publisher("/simulation_data/states", State2d, queue_size = 1)

        #publisher setup for 2d orbit
        self.pubOrbitParams = rospy.Publisher("/simulation_data/orbit_params", OrbitMsg, queue_size = 1)

        #debug publishers
        self.pubEccentrVector = rospy.Publisher("/debug/e_vector", Vector3, queue_size = 1)

    def applyThrust(self, delta_v):
        self.currentState[0,2] += delta_v[0, 0]
        self.currentState[0,3] += delta_v[0, 1]

    def getStates(self):
        return (self.currentState[0,0], self.currentState[0,1], self.currentState[0,2], self.currentState[0,3])

    def publish_states(self, event=None):
        pos_x, pos_y, vel_x, vel_y = self.getStates()

        state = State2d()
        state.position = Vector3(pos_x, pos_y, 0)
        state.velocity = Vector3(vel_x, vel_y, 0)

        self.pubSimulationData.publish(state)
        return

    def publish_orbit_params(self, event=None):
        #calculate orbit params from current state
        self.orbit.updateOrbitParams(self)
        a, e, w, theta = self.orbit.getOrbitParams()

        #Create orbit 2d instace and publish
        orbit2d = OrbitMsg()
        orbit2d.a_orbit = a
        orbit2d.e_orbit = e
        orbit2d.theta_orbit = theta
        orbit2d.w_orbit = w
        self.pubOrbitParams.publish(orbit2d)

        #debug: publish e vector
        e_vector = self.orbit.e_vector
        self.pubEccentrVector.publish(Vector3(e_vector[0,0], e_vector[0,1], e_vector[0,2]))

class Orbit2d(object):
    def __init__(self, spacecraft = None):
        self.a_orbit = None
        self.e_orbit = None
        self.e_vector = None
        self.theta_orbit = None
        self.w_orbit = None
        if spacecraft:
            self.id = spacecraft.id
        else:
            self.id = None

    def calculateEccVec(self):
        return self.e_orbit*np.array([[np.cos(self.w_orbit), np.sin(self.w_orbit)]])

    def updateOrbitParams(self, spacecraft):
        ### calculate keplerian orbit parameters from state space (r, v)

        #auxiliary vector
        x, y, v_x, v_y = spacecraft.getStates()
        radius = np.sqrt(x**2 + y**2)
        h = x*v_y - y*v_x
        h_vector = np.array([[0, 0, h]])
        v_vector = np.array([[v_x, v_y, 0]])
        r_vector = np.array([[x, y, 0]])
        e_vector = np.cross(v_vector, h_vector)/Solver2d.mi - r_vector/radius

        #semi-major axis
        self.a_orbit = Solver2d.mi*radius/(2*Solver2d.mi - radius*(v_x**2 + v_y**2))

        #eccentricity
        #self.e_orbit = np.sqrt(1 - h**2/(Solver2d.mi*self.a_orbit))

        #eccentricity
        self.e_orbit = np.linalg.norm(e_vector)

        #omega
        if e_vector[0,1] < 0:
            self.w_orbit = np.arccos(e_vector[0,0]/self.e_orbit)
        else:
            self.w_orbit = 2*math.pi - np.arccos(e_vector[0,0]/self.e_orbit)

        #true anomaly
        u_vector = np.cross(h_vector, e_vector)
        if np.dot(u_vector[0], r_vector[0]) > 0:
            self.theta_orbit = np.arccos(np.dot(e_vector[0], r_vector[0])/(radius*self.e_orbit))
        else:
            self.theta_orbit = 2*math.pi - np.arccos(np.dot(e_vector[0], r_vector[0])/(radius*self.e_orbit))

        #debug
        self.e_vector = e_vector

    def getOrbitParams(self):
        return (self.a_orbit, self.e_orbit, self.w_orbit, self.theta_orbit)

class Maneuever2d(object):
    pass

