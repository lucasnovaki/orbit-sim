import numpy as np

class Solver():

    grav_const = 6.674e-20 # km^3/(kg s^2)
    earth_mass = 5.9722e24 # kg
    mi = grav_const*earth_mass # km^3/s^2
    r_pe = 15150.0 #km
    r_apo = 56070.0 #km
    a_orbit = (r_pe + r_apo)/2 #km
    v_pe = np.sqrt(mi*(2/r_pe - 1/a_orbit))

    def __init__(self, mode = 'euler', dt = 1, initState = np.array([[0, -15150.0, 6.436, 0]]), buffer_size = 100):
        
        #solver variables
        self.dt = dt
        self.currentTime = 0
        self.initState = initState
        self.currentState = initState

        #integration method
        if mode == 'euler': self.integrate = self.euler
        elif mode == 'runge-kutta': self.integrate = self.rungekutta
        else: 
            self.integrate = self.euler
            self.mode = 'euler'

        #initialize state history
        #self.stateHistory = [None]*buffer_size
        #self.stateHistory[0] = self.initState

    def step(self):

        #get derivative
        self.diff = self.getDerivative()

        #integration
        self.currentState = self.integrate()

        #update time
        self.currentTime += self.dt

        return

    def getDerivative(self):

        #so it is easier to manipulate
        x = self.currentState[0,0]
        y = self.currentState[0,1]
        dx = self.currentState[0,2]
        dy = self.currentState[0,3]

        #calculate rotation matrix from current state
        theta = np.arctan2(y, x)
        rotMatrix = self.getRotMatrix(theta)

        #derivative in radial/tangencial frame
        ddxTang = -np.array([[Solver.mi/(x**2 + y**2)], [0]])

        #get derivative in global frame
        diffGlobal = np.vstack(( np.array([[dx],[dy]]), np.matmul(rotMatrix, ddxTang) )).transpose()

        return diffGlobal

    def getRotMatrix(self, alpha):
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha),np.cos(alpha)]])

    def getStates(self):
        return (self.currentState[0,0], self.currentState[0,1], self.currentState[0,2], self.currentState[0,3])

    def euler(self):
        return self.currentState + self.diff*self.dt

    def rungekutta(self):
        pass