from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff
import numpy as np


class BF(object):
    """ BF class for defining barrier function

    Args:
        h (list): lambdafied expression #! Verify This
        B (list): lambdafied expression #! Verify This
        LHS (list): lambdafied expression #! Verify This
        RHS (list): lambdafied expression #! Verify This
    """

    def __init__(self, h=[], B=[]):
        self.h = h
        self.B = B
        self.LHS = []
        self.RHS = []


class CBF(object):
    """CBF class for deBF_dfining control barrier functions

    Args:
        h (list): lambdafied expression #! Verify This
        B (list): lambdafied expression #! Verify This
        ego (system): ego system
        agent (system): agent system
    """

    def __init__(self, h, B, ego, agent):
        self.states = [ego.states, agent.states]
        self.BF = BF(h, B)
        self.compute_LHS_RHS(ego, agent)
        self.agent = agent

    def compute_LHS_RHS(self, ego, agent):
        """
        Computes "B_dot <= -alpha*B(x)" if B<=0 is safe
        LHS*ego.inputs <= RHS

        Args:
            ego <class System>
            agent <class System>
        """
        # TODO: Extend for stochastic (there is additional term there)
        # TODO: Extend for risk bounds (extra constraints)

        alpha = 1
        BFsym = self.BF.B(*self.states)
        BF_d = BFsym.diff(Matrix([ego.states, agent.states]))
        self.BF.RHS = lambdify([ego.states, agent.states, agent.inputs],
                               - alpha * BFsym - (BF_d.T * Matrix([ego.f, agent.f]))[0])
        self.BF.LHS = lambdify([ego.states, agent.states],
                               (Matrix(BF_d[:ego.nDim]).T * ego.g))

        # BF_d2 =  self.BF.diff(self.x_o_s,2)
        # UnsafeInfo.CBF = lambdify([ego.states,self.x_o_s], CBF)

    def details(self):
        return '{}\n {}\n {}\n'.format(self.BF.h(*self.states), self.BF.B(*self.states), self.states)


class Map_CBF(object):
    def __init__(self, env_bounds, ego):
        # TODO: add checks on the passed argument
        """
        Computes "B_dot <= -alpha*B(x)" if B<=0 is safe
        LHS*ego.inputs <= RHS

        Args:
            env_bounds: object that has either or all of these attributes {'x_min','x_max','y_min','y_max',''}
            ego <class System>
        """

        self.states = ego.states
        self.BF = BF()

        alpha = 2

        for attr in ["x_min", "x_max", "y_min", "y_max"]:
            if hasattr(env_bounds, attr):
                if attr == "x_min":
                    h = -(-ego.states[0] + getattr(env_bounds, attr))
                elif attr == "x_max":
                    h = -(ego.states[0] - env_bounds.x_max)
                elif attr == "y_min":
                    h = -(-ego.states[1] + env_bounds.y_min)
                elif attr == "y_max":
                    h = -(ego.states[1] - env_bounds.y_max)
                CBF = -h
                BF_d = CBF.diff(Matrix([ego.states]))
                self.BF.h.append(lambdify([ego.states], h))
                self.BF.B.append(lambdify([ego.states], CBF))
                self.BF.RHS.append(
                    lambdify([ego.states], -alpha * CBF - (BF_d.T * ego.f)[0]))
                self.BF.LHS.append(lambdify([ego.states], (BF_d.T * ego.g)))

    def add_map_cbf():
        return sympyMatrix


class Goal_Lyap(object):
    def __init__(self, goal_center, goal_set_func, ego):
        self.set = goal_set_func
        self.goal_center = goal_center
        GoalSym = goal_set_func(ego.states)
        self.Lyap = lambdify([ego.states, ego.inputs],
                             GoalSym.diff(ego.states).T * ego.dx)
