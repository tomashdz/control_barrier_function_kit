from sympy import Symbol, symbols, Matrix, sin, cos, exp
from system import *
from controller import *
from CBF import *
import numpy as np

#TODO: (Tom) this should be temporal
def strList2SympyMatrix(str_list):
    sympySymbols = []
    for istr in str_list:
        sympySymbol = Symbol(istr)
        sympySymbols.append(sympySymbol)
    sympyMatrix = Matrix(sympySymbols)
    return sympyMatrix

class Model(object):
    def __init__(self, states, inputs):
        self.states = states
        self.inputs = inputs
        return


class Appr_unicycle_model(Model):
    """This function defines approximate unicycle model 

    Args:
        states_str (list): name list of system states 
        inputs_str (list): name list of system inputs

    Raises:
        ValueError: Raised when l = value is not given in the kwargs or 
                    when the size of states and inputs do not match the model

    Returns:
        f, g, dx (symbolic expressions): to describe model of the system as dx = f+g*input
    """

    def __init__(self, states, inputs, l):
        if states.shape[0] != 3 or inputs.shape[0] != 2:
            raise ValueError("appr_unicycle model has 3 states and 2 inputs")

        super(Appr_unicycle_model, self).__init__(states, inputs)

        self.f = Matrix([0,0,0])    #TODO: (Tom) why here is empty?
        self.g = Matrix([[cos(self.states[2]), -l*sin(self.states[2])], [sin(self.states[2]), l*cos(self.states[2])], [0, 1]])
        self.dx = self.f+self.g*self.inputs
        return


class  Agent_break_model(Model):
    """This function defines agent model with the assumption that the agent maintains its velocities
    in the x and y direction unless it is close to the ego when it slows down 

    Args:
        states (Sympy matrix): vector of symbolic system states 
        inputs (Sympy matrix): vector of symbolic system inputs

    Raises:
        ValueError: Raised when value of radi and c is not given in the kwargs or 
                    when the size of states and inputs do not match the model

    Returns:
        f, dx (symbolic expressions): to describe model of the system as dx = f
    """
    def __init__(self, states, inputs, radi, multi):
        if states.shape[0] != 4 or inputs.shape[0] != 2:
            raise ValueError("appr_unicycle model has 3 states and 2 inputs")

        super(Agent_break_model, self).__init__(states, inputs)

        c = multi
        self.f = Matrix([states[2],states[3],-exp( c*(radi-(states[0]-inputs[0])**2) ),  -exp( c*(radi-(states[1]-inputs[1])**2) )] )
        self.dx = self.f
        return



if __name__ == '__main__':
    """This is main
    """
    # EGO
    states_str = ['xr_0', 'xr_1', 'xr_2']
    inputs_str = ['ur_0', 'ur_1']
    l = 0.1

    states = strList2SympyMatrix(states_str)
    inputs = strList2SympyMatrix(inputs_str)

    ego_model = Appr_unicycle_model(states, inputs, l)

    ego_controller = Controller([[1,0,0],[0,1,0]])

    ego_system = System('ego', ego_model.states, ego_model.inputs, ego_model, ego_controller)
    print(ego_system.system_details())


    # AGENT
    states_str = ['xo_0', 'xo_1', 'xo_2', 'xo_3']
    inputs_str = ['ur_0', 'ur_1']   #TODO: (Tom) can we use same name of symbol?

    states = strList2SympyMatrix(states_str)
    inputs = strList2SympyMatrix(inputs_str)

    agent_model = Agent_break_model(states, inputs, 1, 10)


    G = np.eye(len(states))
    agent_controller = Controller([[1,0,0,0],[0,1,0,0]])
    D = np.eye(2)

    agent_system = Stochastic('agent', states, inputs, agent_model, agent_controller, G, D )
    print(agent_system.system_details())
    UnsafeRadius = 0.5
    h = lambda x, y : (x[0]-y[0])**2+(x[1]-y[1])**2-(UnsafeRadius+l)**2
    CBFs = CBF(h,[ego_system.states, agent_system.states])



## This may help in the future for creating a library of models:
# def  modelinfo(modelname, states, inputs, **kwarg):
#     keys = ["appr_unicycle", "nimble_ant"]

#     if modelname == "appr_unicycle":
#         if len(states) != 3 or len(inputs)!=2:
#                 raise ValueError("appr_unicycle model has 3 states and 2 inputs")
#         for key, value in kwargs.items():
#             if key == "l":
#                 l = value
#         try: l
#         except NameError: ValueError('you need to define l for this model')
#         else:
#             f = Matrix([0,0,0])
#             g = Matrix([[cos(states[2]), -l*sin(states[2])], [sin(states[2]), l*cos(states[2])], [0, 1]])
#             dx = f+g*inputs
#     elif modelname == "nimble_ant":
#         if len(states) != 2 or len(inputs) != 2: raise ValueError("nimble_ant model has 2 states")
#         f = Matrix([0,0])
#         g = Matrix([1,1])
#         dx = f+g*inputs
#     else:
#         try: locals()[modelname](states, inputs, *arg)
#         except NameError: ValueError("model not defined")
#         else:
#             f,g,dx = locals()[modelname](states, inputs, *arg)

#     return f,g,dx
##

