from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff
import numpy as np


class CBF(object):
    def __init__(self, h, BF, ego, agent):
        self.h = h
        self.BF = BF
        self.states = [ego.states, agent.states]
        self.compute_LHS_RHS(ego, agent)
        self.agent = agent
        # self.B = exp(-gamma*h)


    def compute_LHS_RHS(self, ego , agent):
        #TODO: Extend for stochacic (there is additional term there)
        #TODO: Extend for risk bounds (extra constraints)

        """
        Computes "B_dot <= -alpha*B(x)" if B<=0 is safe
        LHS*ego.inputs <= RHS

        Args:
            ego <class System>
            agent <class System>
        """
        alpha = 1
        BFsym = self.BF(*self.states)
        BF_d = BFsym.diff(Matrix([ego.states,agent.states]))
        self.RHS = lambdify([ego.states, agent.states, agent.inputs], -alpha*BFsym-(BF_d.T*Matrix([ego.f,agent.f]))[0])
        self.LHS = lambdify([ego.states,agent.states], (Matrix(BF_d[:ego.nDim]).T*ego.g))

        # BF_d2 =  self.BF.diff(self.x_o_s,2)
        # UnsafeInfo.CBF = lambdify([ego.states,self.x_o_s], CBF)

    def details(self):
        return '{}\n {}\n {}\n'.format(self.h(*self.states), self.BF(*self.states), self.states)


class Map_CBF(object):
    def __init__(self, env_bounds, ego):
        #TODO: add checks on the passed argument
        """
        Computes "B_dot <= -alpha*B(x)" if B<=0 is safe
        LHS*ego.inputs <= RHS

        Args:
            env_bounds: object that has either or all of these attributes {'x_min','x_max','y_min','y_max',''}
            ego <class System>
        """

        self.states = ego.states
        self.h = []
        self.BF = []
        self.LHS = []
        self.RHS = []
        alpha = 1

        if hasattr(env_bounds,'x_min'):
                h = -(-ego.states[0]+env_bounds.x_min)   # h(x)<=0 defines unsafe region
                CBF = -h
                BF_d = CBF.diff(Matrix([ego.states]))
                self.h.append(lambdify([ego.states], h))
                self.BF.append(lambdify([ego.states],CBF))
                self.RHS.append(lambdify([ego.states], -alpha*CBF-(BF_d.T*ego.f)[0]))
                self.LHS.append(lambdify([ego.states], (BF_d.T*ego.g)))
        if hasattr(env_bounds,'x_max'):
                h = -(ego.states[0]-env_bounds.x_max)
                CBF = -h
                BF_d = CBF.diff(Matrix([ego.states]))
                self.h.append(lambdify([ego.states], h))
                self.BF.append(lambdify([ego.states],CBF))
                self.RHS.append(lambdify([ego.states], -alpha*CBF-(BF_d.T*ego.f)[0]))
                self.LHS.append(lambdify([ego.states], (BF_d.T*ego.g)))
        if hasattr(env_bounds,'y_min'):
                h = -(-ego.states[1]+env_bounds.y_min)
                CBF = -h
                BF_d = CBF.diff(Matrix([ego.states]))
                self.h.append(lambdify([ego.states], h))
                self.BF.append(lambdify([ego.states],CBF))
                self.RHS.append(lambdify([ego.states], -alpha*CBF-(BF_d.T*ego.f)[0]))
                self.LHS.append(lambdify([ego.states], (BF_d.T*ego.g)))
        if hasattr(env_bounds,'y_max'):
                h = -(ego.states[1]-env_bounds.y_max)
                CBF = -h
                BF_d = CBF.diff(Matrix([ego.states]))
                self.h.append(lambdify([ego.states], h))
                self.BF.append(lambdify([ego.states],CBF))
                self.RHS.append(lambdify([ego.states], -alpha*CBF-(BF_d.T*ego.f)[0]))
                self.LHS.append(lambdify([ego.states], (BF_d.T*ego.g)))        # if hasattr(env_bounds,'f'):
        #         pass #To be filled later

class Goal_Lyap(object):
    def __init__(self,goal_set_func,ego):
        self.set = goal_set_func
        GoalSym = goal_set_func(ego.states)
        self.Lyap = lambdify([ego.states,ego.inputs],GoalSym.diff(ego.states).T*ego.dx)