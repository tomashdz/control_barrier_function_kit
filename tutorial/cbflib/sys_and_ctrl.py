import cvxopt as cvxopt
import numpy as np


def nimble_ant_c(t, x, u, params):
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
    ctrl_param_k = params['ctrl_param_k']
    my_CBF = params['CBF']

    # Reference controller
    #! note that u, the input to the controller, is the state of the system
    u_ref = ctrl_param_k * ((x_goal-u))

    if np.all((u == 0)):
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

    G, h = my_CBF.compute_G_h(u)

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
    # dynamics
    dx0 = u[0]
    dx1 = u[1]

    return [dx0, dx1]

def nimble_ant_with_agent_output(t, x, u, params):
    return x

def nimble_ant_with_agent_f(t, x, u, params):
    """ Agent that moves to the left

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
    # dynamics
    dx0 = u[0]
    dx1 = u[1]
    dx2 = -0.5
    dx3 = 0

    return [dx0, dx1, dx2, dx3]

def nimble_ant_with_agent_c(t, x, u, params):
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
    ctrl_param_k = params['ctrl_param_k']
    my_CBF = params['CBF']

    # Reference controller
    #! note that u, the input to the controller, is the state of the system
    u_ref = ctrl_param_k * ((x_goal-u[0:2]))

    if np.all((u == 0)):
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

    print(u)
    G, h = my_CBF.compute_G_h(u)

    G = cvxopt.matrix(G)
    h = cvxopt.matrix(h)

    # Run optimizer and return solution
    sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
    x_sol = sol['x']
    return x_sol[0:2]