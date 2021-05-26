from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff
import numpy as np


class System(object):

    """
    Args:
        object (system class): creates object of a system, and includes methods for modifying it

    Returns:
        system object: the model describes dx = f(x) + g(x)*inputs , y = Cx where x is the system states
    """
    ####TODO:TODO: check whether you need Matrix here or not

    def __init__(self, name, states, inputs, f, g = None, C = None, connection = None): 
        # TODO: Check the observability given C, the assert part may need more attention too
        Full_states = True          # If true the states are fully and precisely meaurable and y = x
        self.nDim = len(states)
        self.name = name        # TODO: Do we need name??
        self.states = states
        self.inputs = inputs
        self.f = f
        self.state_traj = []
        self.control_traj = []
        if g is not None:
            self.g = Matrix(g)
            try: 
                self.f+self.g*self.inputs
            except: 
                raise ValueError("Inappropriate g or inputs sizes")
            self.dx = self.f+self.g*self.inputs
        else:
            self.g = None
            self.dx = self.f
         # TODO: Check the observability given C, the assert part may need more attention too   
        if C is None:
            self.C = C
        else:
            if np.array(C).shape != np.eye(self.nDim).shape or not p.allclose(np.eye(self.nDim),C):
                assert np.array(C).shape[1] == self.nDim, "inappropriate C shape"   #y = CX
                self.C = Matrix(C)
                Full_states = False

        self.Full_states = Full_states
    
    def add_state_traj(self, state, time):
        self.state_traj.append([time, state[:]])

    def add_control_traj(self, control, time):
        self.control_traj.append([time, command[:]])

    def system_details(self):
        return '{}\n {}\n {}\n {}\n {}\n {}\n {}\n'.format(self.name, self.states, self.inputs, self.f, self.g, self.C, self.Full_states)

class Stochastic(System):
    def __init__(self, name, states, inputs, f, g = None, C = None, G = None, D= None): # G, and D
        super(Stochastic, self).__init__(name, states, inputs, f, g , C)
        nDim = len(self.states)
        if G is None and D is None:
            raise ValueError("Did you mean to create a deterministic system?")
        
        if G is not None:
            assert np.array(G).shape[0] == nDim, "inappropriate G shape"   #dx = f(x)+Gdw
            self.G = Matrix(G)
        else:
            self.G = G
        if D is not None:
            if self.C is None:
                self.C = np.eye(nDim)
            assert np.array(D).shape[0] == self.model.C.shape[0]
            self.D = Matrix(D)
            self.Full_states = False 


    
    def system_details(self):
        superOut = super(Stochastic, self).system_details()
        out = superOut + '{}\n {}\n'.format(self.D, self.G)
        return out




class Connected_system(System):
    def __init__(self,system,connection):
        self.system = system
        self.connection = connection

    def tOdometry_callback(self, odometry):
        self.odometry = odometry # this odometry's coodination is \map

    def odometry_callback(self, poseStamped):
        self.poseStamped = poseStamped