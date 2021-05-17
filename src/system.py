import numpy as np


# TODO: Add description to functions

class System(object):

    """
    Args:
        object (system class): creates object of a system, and includes methods for modifying it

    Returns:
        system object: the model describes dx = f(x) + g(x)*inputs , y = Cx where x is the system states
    """
    ####TODO:TODO: check whether you need Matrix here or not

    def __init__(self, name, states, inputs, f, g, C):
        # TODO: Check the observability given C, the assert part may need more attention too
        Full_states = True          # If true the states are fully and precisely meaurable and y = x
        nDim = len(states)
        if C is None:
            Full_states = True          # If true the states are fully and precisely meaurable and y = x
        else:
            if C.shape != np.eye(nDim).shape or not np.allclose(np.eye(nDim), C):
                assert C.shape[1] == nDim, "inappropriate C shape"   #y = CX   #TODO: (TOM) Is that unacceptable input? if so we need to use error.
                Full_states = False

        self.name = name        # TODO: Do we need name??
        self.states = states
        self.inputs = inputs
        self.f = f
        self.g = g
        self.C = C
        self.Full_states = Full_states


    def system_details(self):
        return '{}\n {}\n {}\n {}\n {}\n {}\n {}\n'.format(self.name, self.states, self.inputs, self.f, self.g, self.C, self.Full_states)


class Stochastic(System):
    def __init__(self, name, states, inputs, f, g, C, G, D):
        super(Stochastic, self).__init__(name, states, inputs, f, g, C)
        #TODO: Add checks to make sure G or D are passed to Stochatic
        nDim = len(self.states)
        if G is None and D is None:
            raise ValueError("Did you mean to create a deterministic system?")
        if G is not None:
            #TODO: (TOM) Is that unacceptable input? if so we need to use error.
            assert np.array(G).shape[0] == nDim, "inappropriate G shape"   #dx = f(x)+Gdw
            self.Full_states = False
        if D is not None:
            #TODO: (TOM) Is that unacceptable input? if so we need to use error.
            assert D.shape[0] == C.shape[0]
            self.Full_states = False

        self.G = G
        self.D = D

    def system_details(self):
        superOut = super(Stochastic, self).system_details()
        out = superOut + '{}\n {}\n'.format(self.D, self.G)
        return out