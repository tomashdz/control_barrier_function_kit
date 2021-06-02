import argparse
import rospy
import re
import roslaunch

from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff
from system import *
from CBF import *
from Control_CBF import *
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
    sympySymbols = []
    for istr in str_list:
        sympySymbol = symbols(istr)
        sympySymbols.append(sympySymbol)
    sympyMatrix = Matrix(sympySymbols)
    return sympyMatrix


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

    f = Matrix([0,0,0])
    g = Matrix([[cos(states[2]), -l*sin(states[2])], [sin(states[2]), l*cos(states[2])], [0, 1]])
    return f, g


def  agent_break(states, inputs, radi, multi):
    """This function defines agent model with the assumption that the agent maintains its velocities
    in the x and y direction unless it is close to the ego when it slows down 

    Args:
        states (Sympy matrix): vector of symbolic system states 
        inputs (Sympy matrix): vector of symbolic system inputs

    Returns:
        f (symbolic expressions): to describe model of the system as dx = f
    """
    if states.shape[0] != 4 or inputs.shape[0] != 2:
        raise ValueError("appr_unicycle model has 3 states and 2 inputs")

    c = multi
    f = Matrix([states[2],states[3],-exp( c*(radi-(states[0]-inputs[0])**2) ),  -exp( c*(radi-(states[1]-inputs[1])**2) )] )
    return f





if __name__ == '__main__':
    """This is main
    """
    
    # assume we have read the names of agents from ROS and stored them here

    agentnames = ['agent','agent1']

    states_str = ['xr_0', 'xr_1', 'xr_2']
    inputs_str = ['ur_0', 'ur_1']

    states = strList2SympyMatrix(states_str)
    inputs = strList2SympyMatrix(inputs_str)
    l = 0.1
    f, g = appr_unicycle(states, inputs, l)
    C = Matrix([[1,0,0],[0,1,0]])
    # ego_system = System('ego', states, inputs, f, g)
    ego_system = System('HSR', states, inputs, f, g, C)   
    
    print(ego_system.system_details())

    # AGENTS #
    # agent1
    states_str = ['xo_0', 'xo_1', 'xo_2', 'xo_3']
    inputs_str = ['xr_0', 'xr_1']

    states = strList2SympyMatrix(states_str)
    inputs = strList2SympyMatrix(inputs_str)
    f = agent_break(states, inputs, 1, 10)
    g = None
    C = Matrix([[1,0,0,0],[0,1,0,0]])
    G = Matrix(np.eye(len(states)))
    D = Matrix(np.eye(2))
    # agent = Stochastic('agent',states, inputs, f, None, C, G = G , D= D)
    agent_system = System('agent',states, inputs, f)   
    # One agent instance is enough for all agents of the same type, if we have other types of agents,
    # we can create that, we may need to think about a way to assign agents to system
    print(agent_system.system_details())


    # agent2
    agent_system2 = System('agent2',states, inputs, f)   
    # One agent instance is enough for all agents of the same type, if we have other types of agents,
    # we can create that, we may need to think about a way to assign agents to system
    print(agent_system.system_details())



    UnsafeRadius = 0.5
    # Define h such that h(x)<=0 defines unsafe region
    h = lambda x, y, UnsafeRadius : (x[0]-y[0])**2+(x[1]-y[1])**2-(UnsafeRadius+l)**2
    h1 = lambda x, y: h(x,y,UnsafeRadius)
    B = lambda x, y: -h(x,y,UnsafeRadius)
    
    CBF1 = CBF(h1, B, ego_system, agent_system)
    print(CBF1.details())

    CBF2 = CBF(h1, B, ego_system, agent_system2)
    print(CBF2.details())
    
    # Enviroment Bounds
    env_bounds = type('', (), {})()
    env_bounds.y_min = -1.2
    env_bounds.y_max = 1 
    corridorMap = Map_CBF(env_bounds,ego_system)

    # Goal set description
    GoalCenter = np.array([0, 0])
    rGoal = np.power(0.5,2)
    goal_set_func = lambda x: (x[0]-GoalCenter[0])**2+(x[1]-GoalCenter[1])**2-rGoal
    goal_func = Goal_Lyap(goal_set_func,ego_system)



    try:
        rospy.init_node('HSR')
        connected_HSR = Connected_system(ego_system, [agent_system, agent_system2])
        Control_CBF = Control_CBF(connected_HSR, [CBF1, CBF2], goal_func, corridorMap)
        control_priod = 0.05 #[sec] we can change controll priod with this parameter.
        rospy.Timer(rospy.Duration(control_priod), Control_CBF.controller_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass