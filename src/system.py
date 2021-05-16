from sympy import Matrix
import numpy as np


# TODO: Add description to functions

class System(object):

    """
    Args:
        object (system class): creates object of a system, and includes methods for modifying it

    Returns:
        system object: the model describes dx = f(x) + g(x)*inputs , y = Cx where x is the system states
    """

    def __init__(self, name, states, inputs, model, controller):
        self.name = name        # TODO: Do we need name??
        self.states = states
        self.inputs = inputs
        self.model = model
        self.controller = controller
        # TODO: Check the observability given C, the assert part may need more attention too
        self.Full_states = True          # If true the states are fully and precisely meaurable and y = x
        nDim = len(states)
        if self.controller.C.shape != np.eye(nDim).shape or not np.allclose(np.eye(nDim),self.controller.C):
            assert self.controller.C.shape[1] == nDim, "inappropriate C shape"   #y = CX
            self.Full_states = False

    def system_details(self):
        return '{}\n {}\n {}\n {}\n {}\n'.format(self.states, self.inputs, self.Full_states, self.model.__dict__, self.controller.__dict__)


class Stochastic(System):
    def __init__(self, name, states, inputs, model, controller, G, D):
        super(Stochastic, self).__init__(name, states, inputs, model, controller)
        #TODO: Add checks to make sure G or D are passed to Stochatic 
        nDim = len(states)
        assert np.array(G).shape[0] == nDim, "inappropriate G shape"   #dx = f(x)+Gdw
        self.G = G

        #TODO: (Tom) Tom misunderstood the check code
        try: self.controller.C
        except: self.controller.C = np.eye(nDim)
        assert np.array(D).shape[0] == self.controller.C.shape[0]
        self.D = D
        self.Full_states = False
