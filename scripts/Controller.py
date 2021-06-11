# from HSR_control import get_model_srv
from gazebo_msgs.srv import GetWorldProperties, GetModelState, GetModelStateRequest
import numpy as np
import rospy
import matplotlib.pyplot as plt
import cvxopt as cvxopt


def cvxopt_solve_qp(P, q, G=None, h=None, A=None, b=None):
# function to solve the quadratic program
    P = .5 * (P + P.T)  # make sure P is symmetric
    args = [cvxopt.matrix(P), cvxopt.matrix(q)]
    if G is not None:
        args.extend([cvxopt.matrix(G), cvxopt.matrix(h)])
        if A is not None:
            args.extend([cvxopt.matrix(A), cvxopt.matrix(b)])
    cvxopt.solvers.options['show_progress'] = False
    cvxopt.solvers.options['maxiters'] = 100
    sol = cvxopt.solvers.qp(*args)
    if 'optimal' not in sol['status']:
        return None
    return np.array(sol['x']).reshape((P.shape[1],))


class Controller(object):
    """Controller class which take the connected system (ego, CBF_list, ROS) and combine it with map_cbf to generate controller as a qp

    Args:
        connected_system (Connected_system): (ego, CBF_list, ROS)
        goal_func ([type]): [description]
        map_info
        IncludeRadius (int): horizon for CBF consideration

    Returns:
        [type]: [description]
    """
    # make sure ego is the system with which CBF is created
    def __init__(self, connected_system, goal_func, map_info, horizon_radius=10):
        self.connected_system = connected_system
        self.goal_info = goal_func
        self.map_info = map_info
        self.horizon_radius = horizon_radius
        self.control_callback_count = 0  # num of times control_callback is called

    def controller_callback(self, event):
        # this controller loop call back.
        self.control_callback_count += 1

        u = self.cbf_controller_compute()
        self.connected_system.publish(u)

        if self.control_callback_count > 1000:
            rospy.loginfo('reached counter!!')
            rospy.signal_shutdown('reach counter')
        elif self.goal_info.set(self.connected_system.ego.curr_state) < 0:
            rospy.loginfo('reached Goal set!!')
            rospy.signal_shutdown('reached goal_info h')

    def cbf_controller_compute(self):
        ego = self.connected_system.ego
        cbf_list = self.connected_system.cbf_list
        x_r = ego.curr_state
        x_o = []
        for CBF in cbf_list:
            x_o.append(CBF.agent.curr_state)
        u_s = ego.inputs
        cbf_list = cbf_list
        goal_info = self.goal_info
        Map = self.map_info

        unsafe_list = []
        dists = []
        for CBF in cbf_list:
            dist = CBF.BF.h(x_r, CBF.agent.curr_state)
            dists.append(dist)
            if dist < self.horizon_radius:
                unsafe_list.append(CBF)
            ai = 1
        if min(dists) < 0:
            in_unsafe = 1
        else:
            in_unsafe = 0
        minDist = min(dists)
        minJ = np.where(dists == minDist)

        num_qp_vars = len(u_s)+len(unsafe_list)+len(Map.BF.h)+1
        num_constraints = 2*len(u_s)+len(unsafe_list)+len(Map.BF.h)+2

        A = np.zeros((num_constraints, num_qp_vars))
        b = np.zeros(num_constraints)

        for j in range(len(unsafe_list)):
            # CBF Constraints
            x_o = unsafe_list[j].agent.curr_state
            try:
                dx = np.array(unsafe_list[j].agent.state_traj[-5:-1][-1][1]) - \
                    np.array(unsafe_list[j].agent.state_traj[-5:-1][0][1])
                dt = unsafe_list[j].agent.state_traj[-5:-1][-1][0] - \
                    unsafe_list[j].agent.state_traj[-5:-1][0][0]
                mean_inp = dx/dt
            except:
                mean_inp = [0, 0, 0]

            A[j, np.arange(len(u_s))] = unsafe_list[j].BF.LHS(x_r, x_o)[0]
            A[j, len(u_s)+j] = -1
            b[j] = unsafe_list[j].BF.RHS(x_r, x_o, mean_inp)

        # Adding U constraint
        A[len(unsafe_list), 0] = 1
        b[len(unsafe_list)] = ego.input_range[0, 1]
        A[len(unsafe_list)+1, 0] = -1
        b[len(unsafe_list)+1] = -ego.input_range[0, 0]
        A[len(unsafe_list)+2, 1] = 1
        b[len(unsafe_list)+2] = ego.input_range[1, 1]
        A[len(unsafe_list)+3, 1] = -1
        b[len(unsafe_list)+3] = -ego.input_range[1, 0]

        # Adding map constraints
        for j in range(len(Map.BF.h)):
            A[len(unsafe_list) + 2 * len(u_s)+j,
              np.arange(len(u_s))] = Map.BF.LHS[j](x_r)[0]
            A[len(unsafe_list) + 2 * len(u_s)+j, len(u_s)+len(unsafe_list)+j] = -1
            b[len(unsafe_list) + 2 * len(u_s)+j] = Map.BF.RHS[j](x_r)

        # Adding goal_info based Lyapunov !!!!!!!!!!!!!!!!! Needs to be changed for a different example
        A[len(unsafe_list)+ 2 * len(u_s)+len(Map.BF.h),
          0:2] = [goal_info.Lyap(x_r, [1, 0]), goal_info.Lyap(x_r, [0, 1])]
        A[len(unsafe_list)+ 2 * len(u_s)+len(Map.BF.h), -1] = -1
        b[len(unsafe_list)+ 2 * len(u_s)+len(Map.BF.h)] = 0
        A[len(unsafe_list)+ 2 * len(u_s)+len(Map.BF.h)+1, -1] = 1
        b[len(unsafe_list)+ 2 * len(u_s)+len(Map.BF.h)+1] = np.finfo(float).eps + 1

        H = np.zeros((num_qp_vars, num_qp_vars))
        H[0, 0] = 0
        H[1, 1] = 0

        ff = np.zeros((num_qp_vars, 1))
        for j in range(len(unsafe_list)):
            # To reward not using the slack variables when not required
            ff[len(u_s)+j] = 100
            # H[len(u_s)+j,len(u_s)+j] = 100      # To reward not using the slack variables when not required

        for j in range(len(Map.BF.h)):
            H[len(u_s)+len(unsafe_list)+j, len(u_s)+len(unsafe_list)+j] = 1
        ff[-1] = np.ceil(self.control_callback_count/100.0)

        try:
            uq = cvxopt_solve_qp(H, ff, A, b)
            print(uq[1])
        except ValueError:
            uq = [0, 0]
            rospy.loginfo('Domain Error in cvx')

        if uq is None:
            uq = [0, 0]
            rospy.loginfo('infeasible QP')

        return uq
