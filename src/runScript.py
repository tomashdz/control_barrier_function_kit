from sympy import Symbol, symbols, Matrix, sin, cos, exp
from system import *
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


def appr_unicycle(states, inputs, l):
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

    if states.shape[0] != 3 or inputs.shape[0] != 2:
        raise ValueError("appr_unicycle model has 3 states and 2 inputs")

    f = Matrix([0,0,0])    #TODO: (Tom) why here is empty?
    g = Matrix([[cos(states[2]), -l*sin(states[2])], [sin(states[2]), l*cos(states[2])], [0, 1]])
    return f, g


def  agent_break(states, inputs, radi, multi):
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
    if states.shape[0] != 4 or inputs.shape[0] != 2:
        raise ValueError("appr_unicycle model has 3 states and 2 inputs")

    c = multi
    f = Matrix([states[2],states[3],-exp( c*(radi-(states[0]-inputs[0])**2) ),  -exp( c*(radi-(states[1]-inputs[1])**2) )] )
    return f



if __name__ == '__main__':
    """This is main
    """
    # EGO
    states_str = ['xr_0', 'xr_1', 'xr_2']
    inputs_str = ['ur_0', 'ur_1']

    states = strList2SympyMatrix(states_str)
    inputs = strList2SympyMatrix(inputs_str)
    l = 0.1
    f, g = appr_unicycle(states, inputs, l)
    C = Matrix([[1,0,0],[0,1,0]])
    ego_system = System('ego', states, inputs, f, g, C)
    print(ego_system.system_details())


    # AGENT
    states_str = ['xo_0', 'xo_1', 'xo_2', 'xo_3']
    inputs_str = ['xr_0', 'xr_1']

    states = strList2SympyMatrix(states_str)
    inputs = strList2SympyMatrix(inputs_str)
    f = agent_break(states, inputs, 1, 10)
    g = None
    C = Matrix([[1,0,0,0],[0,1,0,0]])
    G = Matrix(np.eye(len(states)))
    D = Matrix(np.eye(2))
    agent_system = Stochastic('agent', states, inputs, f, g, C, G, D)
    print(agent_system.system_details())

    UnsafeRadius = 0.5
    h = lambda x, y : (x[0]-y[0])**2+(x[1]-y[1])**2-(UnsafeRadius+l)**2
    CBFs = CBF(h,[ego_system, agent_system])
