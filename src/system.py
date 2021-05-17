import numpy as np


# TODO: Add description to functions

class System(object):

    """
    Args:
        object (system class): creates object of a system, and includes methods for modifying it

    Returns:
        system object: the model describes dx = f(x) + g(x)*inputs , y = Cx where x is the system states
    """

    def __init__(self, name, states, inputs, f, g, C):
        self.name = name        # TODO: Do we need name??
        self.states = states
        self.inputs = inputs
        self.f = f
        self.g = g
        self.C = C
        # TODO: Check the observability given C, the assert part may need more attention too
        self.Full_states = True          # If true the states are fully and precisely meaurable and y = x
        nDim = len(states)
        if self.C.shape != np.eye(nDim).shape or not np.allclose(np.eye(nDim),self.C):
            assert self.C.shape[1] == nDim, "inappropriate C shape"   #y = CX
            self.Full_states = False

    def system_details(self):
        return '{}\n {}\n {}\n {}\n {}\n {}\n {}\n'.format(self.name, self.states, self.inputs, self.f, self.g, self.C, self.Full_states)


class Stochastic(System):
    def __init__(self, name, states, inputs, f, g, C, G, D):
        super(Stochastic, self).__init__(name, states, inputs, f, g, C)
        self.D = D
        self.G = G
        #TODO: Add checks to make sure G or D are passed to Stochatic 
        #TODO: (Tom) Tom misunderstood the check code
        #nDim = len(states)
        #assert np.array(G).shape[0] == nDim, "inappropriate G shape"   #dx = f(x)+Gdw
        #self.C = np.eye(nDim):
        #    assert np.array(D).shape[0] == self.C.shape[0]
        #    self.Full_states = False

    def system_details(self):
        superOut = super(Stochastic, self).system_details()
        out = superOut + '{}\n {}\n'.format(self.D, self.G)
        return out