import numpy as np
from geometry_msgs.msg import Vector3
from orbit_sim.msg import State2d, Orbits, BodyID
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

    def __init__(self, dt):
        #solver variables
        self.dt = dt
        self.currentTime = 0

        #spacecrafts
        self.lst_ids = list()
        self.spacecrafts = dict()

    def step(self, event=None):

        for id in self.lst_ids:
            self.spacecrafts[id] = self.integrate(self.spacecrafts[id])

        #update time
        self.currentTime += self.dt

    def integrate(self, sc):
        #integrates states of spacecraft and returns updated object

        #get derivative
        diff = self.calculateDerivative(sc)

        #integration
        sc.currentState = sc.currentState + diff*self.dt

        return sc

    def calculateDerivative(self, sc):

        #so it is easier to manipulate
        x = sc.currentState[0,0]
        y = sc.currentState[0,1]
        dx = sc.currentState[0,2]
        dy = sc.currentState[0,3]

        #calculate rotation matrix from current state
        theta = np.arctan2(y, x)
        rotMatrix = self.getRotMatrix(theta)

        #derivative in radial/tangencial frame
        ddxTang = -np.array([[Solver2d.mi/(x**2 + y**2)], [0]])

        #get derivative in global frame
        diffGlobal = np.vstack(( np.array([[dx],[dy]]), np.matmul(rotMatrix, ddxTang) )).transpose()

        return diffGlobal
    
    def add_spacecraft(self, id, initState):
        self.spacecrafts[id] = Spacecraft2d(id, initState)
        self.lst_ids.append(id)
        rospy.loginfo("Spacecraft {:d} added.".format(id))

    def remove_spacecraft(self, id):
        self.lst_ids.remove(id)
        self.spacecrafts.pop(id)
        rospy.loginfo("Spacecraft {:d} deleted.".format(id))

    @classmethod
    def getRotMatrix(cls, alpha):
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha),np.cos(alpha)]])
    
class SolverInterface(Solver2d):

    def __init__(self, dt):

        #initialize 2d solver and inherit methods
        super(SolverInterface, self).__init__(dt)

        #publisher setup for 2d states
        self.pubSimulationData = rospy.Publisher("/simulation_data/states", State2d, queue_size = 1)

        #publisher setup for 2d orbit
        self.pubOrbitParams = rospy.Publisher("/simulation_data/orbit_params", Orbits, queue_size = 1)

        #debug publishers
        self.pubEccentrVector = rospy.Publisher("/simulation_debug/e_vector", Vector3, queue_size = 1)

        #subscriber setup
        self.subSpawnerAdd = rospy.Subscriber("/SpawnControl/spawn", State2d, self.callbackSpawnerAdd)
        self.subSpawnerDelete = rospy.Subscriber("/SpawnControl/delete", BodyID, self.callbackSpawnerDelete)

    def publish_states(self, event=None):

        state = State2d()
        for id in self.lst_ids:
            sc = self.spacecrafts[id]
            pos_x, pos_y, vel_x, vel_y = sc.getStates()

            state.id.append(id)
            state.position.append(Vector3(pos_x, pos_y, 0))
            state.velocity.append(Vector3(vel_x, vel_y, 0))

        if len(self.lst_ids)>0:
            self.pubSimulationData.publish(state)
        return

    def publish_orbit_params(self, event=None):
        orbitsMsg = Orbits()
        for id in self.lst_ids:
            orbitsMsg.id.append(id)
            sc = self.spacecrafts[id]

            #calculate orbit params from current state
            sc.orbit.updateOrbitParams(sc)
            orbit_msg = sc.orbit.toOrbitMsg()

            #append to list
            orbitsMsg.orbit.append(orbit_msg)

            #debug: publish e vector
            #e_vector = sc.orbit.e_vector
        
        if len(self.lst_ids)>0:
            self.pubOrbitParams.publish(orbitsMsg)
            #self.pubEccentrVector.publish(Vector3(e_vector[0,0], e_vector[0,1], e_vector[0,2]))
        return
    
    def callbackSpawnerAdd(self, state_msg):
        if state_msg.id[0] not in self.lst_ids:
            pos = state_msg.position[0]
            vel = state_msg.velocity[0]
            initState = np.array([[pos.x, pos.y, vel.x, vel.y]])
            self.add_spacecraft(state_msg.id[0], initState)

    def callbackSpawnerDelete(self, body_id_msg):
        if body_id_msg.id in self.lst_ids:
            self.remove_spacecraft(body_id_msg.id)


class Spacecraft2d(object):
    def __init__(self, id, initState = np.array([[0, -15150.0, 6.436, 0]])):

        #spacecraft id
        self.id = id
        self.initState = initState
        self.currentState = initState

        #orbit shape
        self.orbit = Orbit2d(self)

    def applyThrust(self, delta_v):
        # delta_v = (dx, dy) in spacecraft reference frame
        theta = np.arctan2(self.currentState[0,1], self.currentState[0,0])
        delta_v = np.matmul(Solver2d.getRotMatrix(theta), delta_v.transpose()).transpose()

        self.currentState[0,2] += delta_v[0, 0]
        self.currentState[0,3] += delta_v[0, 1]

    def getStates(self):
        return (self.currentState[0,0], self.currentState[0,1], self.currentState[0,2], self.currentState[0,3])

class Orbit2d(object):
    def __init__(self, spacecraft = None, orbitMsg = None):

        #main orbital elements
        self.a_orbit = None
        self.e_orbit = None
        self.w_orbit = None
        self.theta_orbit = None

        #auxiliary vectors
        self.e_vector = None
        self.h_vector = None
        
        self.id = None

        if spacecraft:
            self.id = spacecraft.id
            self.updateOrbitParams(spacecraft)

        elif orbitMsg:
            self.setOrbit(orbitMsg)
        

    def calculateEccVec(self):
        return self.e_orbit*np.array([[np.cos(self.w_orbit), np.sin(self.w_orbit)]])

    def updateOrbitParams(self, spacecraft):
        ### calculate keplerian orbit parameters from state space (r, v)

        #auxiliary vector
        x, y, v_x, v_y = spacecraft.getStates()
        radius = np.sqrt(x**2 + y**2)
        h = x*v_y - y*v_x
        self.h_vector = np.array([[0, 0, h]])
        v_vector = np.array([[v_x, v_y, 0]])
        r_vector = np.array([[x, y, 0]])
        e_vector = np.cross(v_vector, self.h_vector)/Solver2d.mi - r_vector/radius

        #semi-major axis
        self.a_orbit = Solver2d.mi*radius/(2*Solver2d.mi - radius*(v_x**2 + v_y**2))

        #eccentricity
        self.e_orbit = np.linalg.norm(e_vector)

        #omega
        if e_vector[0,1] < 0:
            self.w_orbit = np.arccos(e_vector[0,0]/self.e_orbit)
        else:
            self.w_orbit = 2*math.pi - np.arccos(e_vector[0,0]/self.e_orbit)

        #true anomaly
        u_vector = np.cross(self.h_vector, e_vector)
        if np.dot(u_vector[0], r_vector[0]) > 0:
            self.theta_orbit = np.arccos(np.dot(e_vector[0], r_vector[0])/(radius*self.e_orbit))
        else:
            self.theta_orbit = 2*math.pi - np.arccos(np.dot(e_vector[0], r_vector[0])/(radius*self.e_orbit))

        #debug
        self.e_vector = e_vector

    def setOrbit(self, orbit):
        if type(orbit) is tuple:
            self.a_orbit, self.e_orbit, self.w_orbit = orbit
        else:        
            self.a_orbit = orbit.a_orbit
            self.e_orbit = orbit.e_orbit
            self.w_orbit = orbit.w_orbit
        self.e_vector = self.calculateEccVec()

    def getOrbitParams(self):
        return (self.a_orbit, self.e_orbit, self.w_orbit, self.theta_orbit)
    
    def toOrbitMsg(self):
        msg = OrbitMsg()
        msg.a_orbit = self.a_orbit
        msg.e_orbit = self.e_orbit
        msg.w_orbit = self.w_orbit
        if self.theta_orbit:
            msg.theta_orbit = self.theta_orbit
        else:
            msg.theta_orbit = 9999
        return msg

