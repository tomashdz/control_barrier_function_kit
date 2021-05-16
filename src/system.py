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

    # def add_output_info(self, C):
    #     if np.array(C).shape != np.eye(self.nDim).shape or not p.allclose(np.eye(self.nDim),C):
    #         assert np.array(C).shape[1] == self.nDim, "inappropriate C shape"   #y = CX
    #         self.model.C = Matrix(C)
    #         self.Full_states = False

    # @classmethod
    # def set_solver(cls, solver):
    #     cls.solver = solver



class Stochastic(System):
    def __init__(self, name, states, inputs, model, controller, G, D):
        super(Stochastic, self).__init__(name, states, inputs, model, controller)
        #TODO: Add checks to make sure G or D are passed to Stochatic 
        nDim = len(states)
        assert np.array(G).shape[0] == nDim, "inappropriate G shape"   #dx = f(x)+Gdw
        self.model.G = G

        try: self.controller.C
        except: self.controller.C = np.eye(nDim)
        assert np.array(D).shape[0] == self.controller.C.shape[0]
        self.model.D = D
        self.Full_states = False









# class Model(System):
#     def __init__(self,name, sys_type, states, inputs, modelname,**kwargs):
#         super(Model, self).__init__(name, sys_type, states, inputs)
#         # Assume modelname is modelhandle , kwargs is the model parameters



#         models = ["appr_unicycle", "nimble_ant"]
#         self.info = type('',(),{})()
#         if modelname in models:
#             self.modelname = modelname
#             self.args = kwargs
#             self.info.f, self.info.g, self.info.dx = self.modelinfo()
#         else:
#             print("Cannot find this model name. Please define it manually.")


#     def  modelinfo(self):
#         if self.modelname == "appr_unicycle":
#             if len(self.states) != 3 or len(self.inputs)!=2:
#                  raise ValueError("appr_unicycle model has 3 states and 2 inputs")
#             for key, value in self.args.items():
#                 if key == "l":
#                     l = value
#             try: l
#             except NameError: ValueError('you need to define l for this model')
#             else:
#                 f = Matrix([0,0,0])
#                 g = Matrix([[cos(self.states[2]), -l*sin(self.states[2])], [sin(self.states[2]), l*cos(self.states[2])], [0, 1]])
#                 dx = f+g*self.inputs
#         elif self.modelname == "nimble_ant":
#             if len(self.states) != 2 or len(self.inputs) != 2: raise ValueError("nimble_ant model has 2 states")
#             f = Matrix([0,0])
#             g = Matrix([1,1])
#             dx = f+g*self.inputs
#         else:
#             raise ValueError("model not defined")
#         return f,g,dx
#     def system_details(self):
#             return '{} {} {} {} {}'.format(self.name, self.type, self.states, self.modelname, self.info.__dict__, self.args)
