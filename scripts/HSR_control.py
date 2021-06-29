# 2021, Shakiba Yaghoubi, Bardh Hoxha, Tom Yamaguchi, Toyota Motor North America

from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff
from system import *
from CBF import *
from Controller import *
import argparse
import rospy
import re
import time
import numpy as np


# ROS msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetWorldProperties, GetModelState, GetModelStateRequest

# ROS others
import tf



def strList2SympyMatrix(str_list):
    sympy_symbols_lst = []
    for istr in str_list:
        sympy_symbol = symbols(istr)
        sympy_symbols_lst.append(sympy_symbol)
    sympy_matrix = Matrix(sympy_symbols_lst)
    return sympy_matrix


def appr_unicycle(states, inputs, l):
    """This function defines approximate unicycle model

    Args:
        states_str (list): name list of system states
        inputs_str (list): name list of system inputs

    Returns:
        f, g (symbolic expressions): to describe model of the system as dx = f+g*input
    """

    if states.shape[0] != 3 or inputs.shape[0] != 2:
        raise ValueError("appr_unicycle model has 3 states and 2 inputs")

    f = Matrix([0, 0, 0])
    g = Matrix([[cos(states[2]), -l*sin(states[2])],
               [sin(states[2]), l*cos(states[2])], [0, 1]])
    return f, g

def appr_unicycle(states, inputs, l):
    """This function defines approximate unicycle model

    Args:
        states_str (list): name list of system states
        inputs_str (list): name list of system inputs

    Returns:
        f, g (symbolic expressions): to describe model of the system as dx = f+g*input
    """

    if states.shape[0] != 3 or inputs.shape[0] != 2:
        raise ValueError("appr_unicycle model has 3 states and 2 inputs")

    f = Matrix([0, 0, 0])
    g = Matrix([[cos(states[2]), -l*sin(states[2])],
               [sin(states[2]), l*cos(states[2])], [0, 1]])
    return f, g

    
def agent_break(states, inputs, radi, multi):
    """This function defines agent model with the assumption that the agent maintains its velocities
    in the x and y direction unless it is close to the ego when it slows down

    Args:
        states (Sympy matrix): vector of symbolic system states
        inputs (Sympy matrix): vector of symbolic system inputs

    Returns:
        f (symbolic expressions): to describe model of the system as dx = f
    """
    if states.shape[0] != 3 or inputs.shape[0] != 2:
        raise ValueError("agent_break model has 3 states and 3 inputs")

    c = multi
    dx = (states[0]-inputs[0])*exp(c*(radi-(states[0]-inputs[0])**2))
    dy = (states[1]-inputs[1])*exp(c*(radi-(states[1]-inputs[1])**2))
    dtetha = 1/(1+(dy/dx)**2)

    f = Matrix([dx, dy, dtetha])
    return f


def careless_agent(states, inputs):
    """This function defines agent model with the assumption that the agent maintains its velocities
    in the x and y direction unless it is close to the ego when it slows down

    Args:
        states (Sympy matrix): vector of symbolic system states
        inputs (Sympy matrix): vector of symbolic system inputs

    Returns:
        f (symbolic expressions): to describe model of the system as dx = f
    """
    if states.shape[0] != 3 or inputs.shape[0] != 3:
        raise ValueError("careless_agent model has 3 states and 3 inputs")

    dx = inputs[0]
    dy = inputs[1]
    dtetha = inputs[2]

    f = Matrix([dx, dy, dtetha])
    return f


if __name__ == '__main__':
    """This is main
    """
    # assume we have read the names of agents from ROS and stored them here

    agentnames = ['agent', 'agent1']

    states_str = ['xr_0', 'xr_1', 'xr_2']
    inputs_str = ['ur_0', 'ur_1']


    states = strList2SympyMatrix(states_str)
    inputs = strList2SympyMatrix(inputs_str)
    l = 0.05
    f, g = appr_unicycle(states, inputs, l)
    C = Matrix([[1, 0, 0], [0, 1, 0]])
    input_range = np.array([[-0.3, 0.3], [-0.3, 0.3]])
    # ego_system = System('ego', states, inputs, f, g)
    ego_system = System('HSR', states, inputs, f, g, None, input_range)
    print(ego_system.system_details())

    # AGENTS #
    # agent1
    states_str = ['xo_0', 'xo_1', 'xo_2']
    # inputs_str = ['xr_0', 'xr_1']
    inputs_str = ['uo_0', 'uo_1', 'uo_2']

    states = strList2SympyMatrix(states_str)
    inputs = strList2SympyMatrix(inputs_str)
    f = careless_agent(states, inputs)
    g = None
    C = Matrix([[1, 0, 0, 0], [0, 1, 0, 0]])
    G = Matrix(np.eye(len(states)))
    D = Matrix(np.eye(2))
    # agent = Stochastic('agent',states, inputs, f, None, C, G = G , D= D)
    agent_system = System('agent1', states, inputs, f)
    # One agent instance is enough for all agents of the same type, if we have other types of agents,
    # we can create that, we may need to think about a way to assign agents to system
    print(agent_system.system_details())

    # agent2
    agent_system2 = System('agent2', states, inputs, f)
    # One agent instance is enough for all agents of the same type, if we have other types of agents,
    # we can create that, we may need to think about a way to assign agents to system
    print(agent_system.system_details())

    unsafe_radius = 1
    # Define h such that h(x)<=0 defines unsafe region
    def h(x, y, unsafe_radius): return (
        x[0] - y[0])**2 + (x[1] - y[1])**2 - (unsafe_radius + l)**2

    def h1(x, y): return h(x, y, unsafe_radius)
    # B initially negative, so Bdot<= -aB
    

    exp_var = True

    if exp_var == True:
        def B(x, y): return exp(-h(x, y, unsafe_radius))-1
        B_type = "exp"
    else:
        def B(x, y): return -h(x, y, unsafe_radius)
        B_type = "lin"


    cbf1 = CBF(h1, B, B_type, ego_system, agent_system)
    print(cbf1.details())

    cbf2 = CBF(h1, B, B_type, ego_system, agent_system2)
    print(cbf2.details())

    # Enviroment Bounds
    env_bounds = type('', (), {})()
    env_bounds.y_min = -1.2
    env_bounds.y_max = 1
    corridor_map = Map_CBF(env_bounds, ego_system)

    # Goal set description
    goal_center = np.array([0, 0])
    r_goal = np.power(0.5, 2)

    def goal_set_func(x): return (
        x[0]-goal_center[0])**2 + (x[1] - goal_center[1])**2 - r_goal
    goal_func = Goal_Lyap(goal_center, goal_set_func, ego_system)

    try:
        rospy.init_node('HSR')
        connected_HSR = Connected_system(ego_system, [cbf1, cbf2])
        my_cbf_controller = Controller(connected_HSR, goal_func, corridor_map, 5 , "reference_control")
        # [sec] we can change controll priod with this parameter.
        control_priod = 0.05
        time.sleep(1)
        rospy.Timer(rospy.Duration(control_priod),
                    my_cbf_controller.controller_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
