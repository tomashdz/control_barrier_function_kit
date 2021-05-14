from system import *
from CBF import *
import numpy as np


def  appr_unicycle_model(states, inputs, **kwargs):
    """This function defines approximate unicycle model 

    Args:
        states (Sympy matrix): vector of symbolic system states 
        inputs (Sympy matrix): vector of symbolic system inputs

    Raises:
        ValueError: Raised when l = value is not given in the kwargs or 
                    when the size of states and inputs do not match the model

    Returns:
        f, g, dx (symbolic expressions): to describe model of the system as dx = f+g*input
    """
    states = Matrix(states)
    inputs = Matrix(inputs)
    if len(states) != 3 or len(inputs)!=2:
            raise ValueError("appr_unicycle model has 3 states and 2 inputs")
    for key, value in kwargs.items():
        if key == "l":
            l = value
    try: l
    except NameError: ValueError('you need to define l for this model')
    else:
        f = Matrix([0,0,0])
        g = Matrix([[cos(states[2]), -l*sin(states[2])], [sin(states[2]), l*cos(states[2])], [0, 1]])
        dx = f+g*inputs
    return f,g,dx

def  agent_break_model(states, inputs, **kwargs):
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
    states = Matrix(states)
    inputs = Matrix(inputs)
    if len(states) != 4 or len(inputs)!=2:
            raise ValueError("appr_unicycle model has 3 states and 2 inputs")
    for key, value in kwargs.items():
        if key == "radi":
            radi = value
        elif key == "mult":
            c = value
        
    try: radi, c
    except NameError: ValueError('you need to define l for this model')
    else:
        f = Matrix([states[2],states[3],-exp( c*(radi-(states[0]-inputs[0])**2) ),  -exp( c*(radi-(states[1]-inputs[1])**2) )] )
        dx = f
    return f,dx




if __name__ == '__main__':
    """This is main
    """
    # EGO
    xr_0, xr_1, xr_3 = symbols('xr_0 xr_1 xr_2')
    ur_0, ur_1 = symbols('ur_0 ur_1')
    states = [xr_0, xr_1, xr_3]
    inputs = [ur_0, ur_1]
    model = type('',(),{})()
    l = 0.1     
    model.f, model.g, model.dx = appr_unicycle_model(states, inputs, l = l)

    C = [[1,0,0],[0,1,0]]
    ego = System('ego','ego', states, inputs, model, C = C)
    print(ego.system_details())
    

    # AGENT
    xo_0, xo_1, xo_2, xo_3 = symbols('xo_0 xo_1 xo_2 xo_3')
    states = [xo_0, xo_1, xo_2, xo_3]
    inputs = [xr_0, xr_1]   #
    model = type('',(),{})()     
    model.f, model.dx = agent_break_model(states, inputs, radi = 1, mult = 10)

   
    G = np.eye(len(states))
    C = [[1,0,0,0],[0,1,0,0]]
    D = np.eye(2)

    agent = Stochastic('agent','agent', states, inputs, model, C = C, G = G , D= D )
    print(agent.system_details())
    UnsafeRadius = 0.5
    h = lambda x, y : (x[0]-y[0])**2+(x[1]-y[1])**2-(UnsafeRadius+l)**2
    CBFs = CBF(h,[ego.states, agent.states])



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

