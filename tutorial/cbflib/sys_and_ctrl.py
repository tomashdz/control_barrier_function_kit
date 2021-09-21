import cvxopt as cvxopt
import numpy as np
import math
from sympy import symbols

def nimble_ant_c(x, params):
    """ Controller for nimble ant

    Args:
        t (float): [description]
        x (numpy.ndarray): [description]
        u (numpy.ndarray): the input to the controller is the state of the system x
        params (dict): Dict keys:
                        goal_x: the goal or target state
                        bad_sets: list of elippses defining bad sets
                        ctrl_param: parameters for the controller
                        CBF: the CBF object
    Returns:
        cvxopt.base.matrix: the control for the system
    """
    x_goal = params['x_goal']
    ctrl_param = params['ctrl_param']
    my_CBF = params['CBF']

    # Reference controller
    #! note that u, the input to the controller, is the state of the system
    u_ref = ctrl_param * ((x_goal-x[0:2]))

    if np.all((x == 0)):
        return u_ref

    ############################
    # cvxopt quadratic program
    # minimize  0.5 x'Px + q'x
    # s.t       Gx<=h
    ############################

    # P matrix
    P = cvxopt.matrix(np.eye(2))
    P = .5 * (P + P.T)  # symmetric

    # q matrix
    q = cvxopt.matrix(-1 * np.array(u_ref), (2, 1))

    G, h = my_CBF.compute_G_h(x)

    G = cvxopt.matrix(G)
    h = cvxopt.matrix(h)

    # Run optimizer and return solution
    sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
    x_sol = sol['x']
    return x_sol[0:2]

def nimble_ant_f(t, x, u, params):
    """ Function for the nimble_ant system

    Args:
        t (float): [description]
        x (numpy.ndarray): [description]
        u (numpy.ndarray): [description]
        params (dict): Dict keys:
                        goal_x: the goal or target state
                        bad_sets: list of elippses defining bad sets
                        ctrl_param: parameters for the controller
                        CBF: the CBF object 

    Returns:
        list: dx
    """

    x_goal = params['x_goal']
    if (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 <= 0.1**2:
        return [0, 0]

    # compute control given current position
    u = nimble_ant_c(x, params)

    # dynamics
    dx0 = u[0]
    dx1 = u[1]

    return [dx0, dx1]

def nimble_ant_with_agent_c(x, params):
    """ Controller for nimble ant with agent

    Args:
        t (float): [description]
        x (numpy.ndarray): [description]
        u (numpy.ndarray): the input to the controller is the state of the system x
        params (dict): Dict keys:
                        goal_x: the goal or target state
                        bad_sets: list of elippses defining bad sets
                        ctrl_param: parameters for the controller
                        CBF: the CBF object
    Returns:
        cvxopt.base.matrix: the control for the system
    """

    x_goal = params['x_goal']
    ctrl_param = params['ctrl_param']
    my_CBF = params['CBF']

    # Reference controller
    #! note that u, the input to the controller, is the state of the system
    u_ref = ctrl_param * ((x_goal-x[0:2]))

    if np.all((x == 0)):
        return u_ref

    ############################
    # cvxopt quadratic program
    # minimize  0.5 x'Px + q'x
    # s.t       Gx<=h
    ############################

    # P matrix
    P = cvxopt.matrix(np.eye(4))
    P = .5 * (P + P.T)  # symmetric

    # q matrix

    #! TEMPORARY - FIX NEEDED
    # q = cvxopt.matrix(-1 * np.array(u_ref), (2, 1))
    u_ref = np.append(u_ref,[0,0],axis=0)
    q = cvxopt.matrix(-1 * np.array(u_ref), (4, 1))

    G, h = my_CBF.compute_G_h(x)

    G = cvxopt.matrix(G)
    h = cvxopt.matrix(h)

    # Run optimizer and return solution
    sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
    x_sol = sol['x']
    return x_sol[0:2]

def nimble_ant_with_agent_f(t, x, u, params):
    """ Function for nimble ant with agent that moves to the left

    Args:
        t (float): [description]
        x (numpy.ndarray): [description]
        u (numpy.ndarray): [description]
        params (dict): Dict keys:
                        goal_x: the goal or target state
                        bad_sets: list of elippses defining bad sets
                        ctrl_param: parameters for the controller
                        CBF: the CBF object 

    Returns:
        list: dx
    """

    # x_goal = params['x_goal']
    # if (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 <= 0.1**2:
    #     return [0, 0]

    # compute control given current position
    u = nimble_ant_with_agent_c(x, params)

    # dynamics
    dx0 = u[0]
    dx1 = u[1]
    dx2 = -0.5
    dx3 = 0

    return [dx0, dx1, dx2, dx3]

def unicycle_f(t, x, u, params):
    # Function for a silly bug
    # if goal reached, do nothing
    goal_x = params['goal_x']
    if np.linalg.norm(x[0:2]-goal_x) <= 0.1:
        return [0, 0, 0]

    # compute control given current position
    u_0 = unicycle_c(x, params)

    # compute change in xy direction
    dx0 = math.cos(x[2])
    dx1 = math.sin(x[2])
    dx2 = u_0[0]

    return [dx0, dx1, dx2]

def unicycle_c(x, params):
    # Controller for nimble car
    goal_x = params['goal_x']
    bad_sets = params['bad_sets']
    ctrl_param = params['ctrl_param']
    myCBF = params['myCBF']

    # Reference controller
    theta_ref = math.atan((goal_x[1]-x[1])/(goal_x[0]-x[0]))
    uref_0 = ctrl_param[0] * (theta_ref - x[2])

    # math.atan2(sin(theta_ref-x[2]), cos(theta_ref-x[2]))

    ############################
    # cvxopt quadratic program
    # minimize  0.5 x'Px + q'x
    # s.t       Gx<=h
    ############################
    # P matrix
    P = cvxopt.matrix(np.eye(1))
    # P = .5 * (P + P.T)  # symmetric

    # q matrix
    q = cvxopt.matrix(np.array([-1*uref_0]), (1, 1))

    G, h = myCBF.compute_G_h(x)

    G = cvxopt.matrix(G)
    h = cvxopt.matrix(h)

    # Run optimizer and return solution
    # sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
    try:
        sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
        x_sol = sol['x']
    except:
        x_sol = [0]
        print("no sol")
    # print(x, ' G: ', G, ' h: ', h, ' x_sol: ', x_sol)
    return x_sol[0:1]

def unicycle_agent_f(t, x, u, params):
    # Function for a silly bug
    # if goal reached, do nothing
    goal_x = params['goal_x']
    if np.linalg.norm(x[0:2]-goal_x) <= 0.1:
        return [0, 0, 0]

    # compute control given current position
    u_0 = unicycle_c(x, params)

    # compute change in xy direction
    dx0 = math.cos(x[2])
    dx1 = math.sin(x[2])
    dx2 = u_0[0]
    dx3 = -1
    dx4 = 0

    return [dx0, dx1, dx2, dx3, dx4]

def unicycle_agent_c(x, params):
    # Controller for nimble car
    goal_x = params['goal_x']
    bad_sets = params['bad_sets']
    ctrl_param = params['ctrl_param']
    myCBF = params['myCBF']

    # Reference controller
    theta_ref = math.atan((goal_x[1]-x[1])/(goal_x[0]-x[0]))
    uref_0 = ctrl_param[0] * (theta_ref - x[2])

    # math.atan2(sin(theta_ref-x[2]), cos(theta_ref-x[2]))

    ############################
    # cvxopt quadratic program
    # minimize  0.5 x'Px + q'x
    # s.t       Gx<=h
    ############################
    # P matrix
    P = cvxopt.matrix(np.eye(3))
    # P = .5 * (P + P.T)  # symmetric

    # q matrix
    uref_0 = np.append(uref_0,[0,0],axis=0)
    q = cvxopt.matrix(np.array([-1*uref_0]), (3, 1))

    G, h = myCBF.compute_G_h(x)

    G = cvxopt.matrix(G)
    h = cvxopt.matrix(h)

    # Run optimizer and return solution
    # sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
    try:
        sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
        x_sol = sol['x']
    except:
        x_sol = [0]
        print(["No sol" + datetime.datetime.now().time()])
    # print(x, ' G: ', G, ' h: ', h, ' x_sol: ', x_sol)
    return x_sol[0:1]

# def unicycle_c(x, params):
#     # Controller for nimble car
#     goal_x = params['goal_x']
#     bad_sets = params['bad_sets']
#     ctrl_param = params['ctrl_param']
#     myCBF = params['myCBF']

#     # Reference controller
#     theta_ref = math.atan((goal_x[1]-x[1])/(goal_x[0]-x[0]))
#     uref_0 = ctrl_param[0] * (theta_ref - x[2])

#     ############################
#     # cvxopt quadratic program
#     # minimize  0.5 x'Px + q'x
#     # s.t       Gx<=h
#     ############################

#     # P matrix
#     P = cvxopt.matrix(np.eye(1))
#     # P = .5 * (P + P.T)  # symmetric

#     # q matrix
#     q = cvxopt.matrix(np.array([-1*uref_0]), (1, 1))

#     G, h = myCBF.compute_G_h(x)

#     G = cvxopt.matrix(G)
#     h = cvxopt.matrix(h)

#     # Run optimizer and return solution
#     # sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
#     try:
#         sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
#         x_sol = sol['x']
#     except:
#         x_sol = [0]
#         print("QP iteration fail. Trying again...")
#     # print(x, ' G: ', G, ' h: ', h, ' x_sol: ', x_sol)
#     return x_sol[0:1]


# def unicycle_f(t, x, u, params):
#     # Function for a silly bug
#     # if goal reached, do nothing
#     goal_x = params['goal_x']
#     if (x[0] - goal_x[0])**2 + (x[1] - goal_x[1])**2 <= 0.1**2:
#         return [0, 0, 0]

#     # compute control given current position
#     u_0 = unicycle_c(x, params)

#     # compute change in xy direction
#     dx0 = math.cos(x[2])
#     dx1 = math.sin(x[2])
#     dx2 = u_0[0]

#     return [dx0, dx1, dx2]



################ TV Stuff

def nimble_ant_tv_c(x, t, params):
    """ Controller for nimble ant

    Args:
        t (float): [description]
        x (numpy.ndarray): [description]
        u (numpy.ndarray): the input to the controller is the state of the system x
        params (dict): Dict keys:
                        goal_x: the goal or target state
                        bad_sets: list of elippses defining bad sets
                        ctrl_param: parameters for the controller
                        CBF: the CBF object
    Returns:
        cvxopt.base.matrix: the control for the system
    """
    x_goal = params['x_goal']
    ctrl_param = params['ctrl_param']
    my_CBF = params['CBF']

    # Reference controller
    #! note that u, the input to the controller, is the state of the system

    ############################
    # cvxopt quadratic program
    # minimize  0.5 x'Px + q'x
    # s.t       Gx<=h
    ############################

    # P matrix
    P = cvxopt.matrix(np.eye(2))
    # P = .5 * (P + P.T)  # symmetric

    # q matrix
    q = cvxopt.matrix(np.array([0.0, 0]), (2, 1))

    # F_[5,15](||x-x_g||)<=5)
    # h(x) = 5 - ||x-x_g||
    # b(x,t) = y(t) -  ||x-x_g||
    # y(t) = -5/15*t + 10
    # t = symbols('t')
    # my_CBF.B = my_CBF.B_TV.subs('t', time)

    # G, h = my_CBF.compute_G_h(x)
    G, h = my_CBF.compute_G_h(x,t)

    G = cvxopt.matrix(G)
    h = cvxopt.matrix(h)
    print('G: ', G, ' h; ',h)
    # Run optimizer and return solution
    sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
    x_sol = sol['x']
    print('sol: ',x_sol)
    
    return x_sol[0:2]

def nimble_ant_tv_f(t, x, u, params):
    """ Function for the nimble_ant system

    Args:
        t (float): [description]
        x (numpy.ndarray): [description]
        u (numpy.ndarray): [description]
        params (dict): Dict keys:
                        goal_x: the goal or target state
                        bad_sets: list of elippses defining bad sets
                        ctrl_param: parameters for the controller
                        CBF: the CBF object 

    Returns:
        list: dx
    """

    x_goal = params['x_goal']
    if (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 <= 0.1**2:
        return [0, 0]

    # compute control given current position
    u = nimble_ant_tv_c(x, t, params)

    # dynamics
    dx0 = u[0]
    dx1 = u[1]

    return [dx0, dx1]