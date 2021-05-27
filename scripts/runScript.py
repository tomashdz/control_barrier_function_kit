import argparse
import rospy
import re

from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff
from system import *
from CBF import *
# from Control_CBF import *
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


# def HSR_connections():
#         # Reading for HSR
#     HSRconnection = type('', (), {})()
#     # HSRconnection.vw_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
#     # subscriber for Gazebo info.
#     # rospy.wait_for_service ('/gazebo/get_model_state')
#     # HSRconnection.get_model_pro = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
#     # HSRconnection.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
#     # HSRconnection.tOdometry_subscriber = rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, tOdometry_callback, queue_size=10)
#     HSRconnection.tOdometry = Odometry()
#     # HSRconnection.odometry_subscriber = rospy.Subscriber('/global_pose', PoseStamped, odometry_callback, queue_size=10)
#     HSRconnection.poseStamped = PoseStamped()
#     # listener of tf.
#     HSRconnection.tfListener = tf.TransformListener()
#     return HSRconnection

# def agent_connections():
#     rospy.wait_for_service ('/gazebo/get_model_state')
#     Agentconnection = type('', (), {})()
#     Agentconnection.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
#     Agentconnection.set_model_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

#     Agentconnection.model = GetModelStateRequest()
#     Agentconnection.model.model_name = model_name
#     return Agentconnection

def tOdometry_callback(odometry):
    #TODO: you need to implement here to transfer the subscribed data to somewhere you want to use.
    #self.odometry = odometry # this odometry's coodination is \map
    Control_CBF.update_odometry(odometry)
    pass


def odometry_callback(poseStamped):
    #TODO: you need to implement here to transfer the subscribed data to somewhere you want to use.
    Control_CBF.update_poseStamped(poseStamped)
    pass


if __name__ == '__main__':
    """This is main
    """

    # # subscliber to get odometry of HSR
    rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, tOdometry_callback(System), queue_size=10)
    rospy.Subscriber('/global_pose', PoseStamped, odometry_callback, queue_size=10)

    # # publisher to send vw order to HSR
    vw_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)

    # # subscriber for agents data from Gazebo info.
    rospy.wait_for_service ('/gazebo/get_model_state')
    get_model_pro = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    actors = []
    model_properties = get_model_pro()
    for model_name in model_properties.model_names:
            if re.search('actor*', model_name) and not model_name in actors:  # if the model name is actor*, it will catch them.
                    actors.append(model_name) 

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
    # agent = Stochastic('agent',states, inputs, f, None, C, G = G , D= D)
    agent_system = System('human',states, inputs, f)   
    #One agent instance is enough for all agents of the same type, if we have other types of agents,
    #we can create that, we may need to think about a way to assign agents to system
    print(agent_system.system_details())

    # try:
    #     rospy.init_node('agent_system')
    #     agent = Agent()
    #     rospy.Timer(rospy.Duration(1.0/freq), agent.control_callback)
    #     rospy.spin()



    #     rospy.init_node('cbf_controller')
    #     rospy.init_node(args.model_name[0]+'_controller')
    #     agent = Agent(args.model_name[0])
    #     rospy.Timer(rospy.Duration(1.0/freq), agent.control_callback)
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass




    UnsafeRadius = 0.5
    # Define h such that h(x)<=0 defines unsafe region
    h = lambda x, y, UnsafeRadius : (x[0]-y[0])**2+(x[1]-y[1])**2-(UnsafeRadius+l)**2
    h1 = lambda x, y: h(x,y,UnsafeRadius)
    B = lambda x, y: -h(x,y,UnsafeRadius)
    CBF1 = CBF(h1, B, ego_system, agent_system)
    print(CBF1.details())

    h = lambda x, minx: (x[0]-minx)
    h = lambda x, maxx: (maxx-x[0])
    # B = ((xr0 - cx)/rad_x)**2 + ((xr1 - cy)/rad_y)**2 - 1

    # Goal set description
    GoalCenter = np.array([0, 0])
    rGoal = np.power(0.5,2)
    goal_set_func = lambda x: (x[0]-GoalCenter[0])**2+(x[1]-GoalCenter[1])**2-rGoal

   


    try:
        rospy.init_node('HSR')
        connected_HSR = Connected_system(ego_system, HSR_connections())
        Control_CBF = Control_CBF(connected_HSR, CBF1, goal_set_func)
        control_priod = 0.05 #[sec] we can change controll priod with this parameter.
        rospy.Timer(rospy.Duration(control_priod), Control_CBF.controller_callback())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    # try:
    #     rospy.init_node('human')
    #     connected_agent = Connected_system(agent_system, agent_connections())
    #     rospy.Timer(rospy.Duration(1.0/freq), Control_CBF.control_callback)
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass