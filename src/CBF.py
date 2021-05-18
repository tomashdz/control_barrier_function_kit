from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff
import numpy as np


class CBF(object):
    def __init__(self, h, BF, ego, agent):
        self.h = h
        self.BF = BF
        self.states = [ego.states, agent.states]
        self.compute_LHS_RHS(ego, agent)
        
        # self.B = exp(-gamma*Uset)


    def compute_LHS_RHS(self, ego , agent):
        """
        B_dot <= -alpha*B(x) if B<=0 is safe
        LHS*inputs <= RHS

        Args:
            agent ([type]): [description]
        """
        alpha = 1
        # h_inp = [ego.states, agent.states]
        BFsym = self.BF(*self.states)
        BF_d = BFsym.diff(Matrix([ego.states,agent.states]))
        self.LHS = lambdify([ego.states,agent.states], -alpha*BFsym-(BF_d.T*Matrix([ego.model.f,agent.model.f]))[0])
        self.RHS = lambdify([ego.states,agent.states], (Matrix(BF_d[:ego.nDim]).T*ego.model.g)[0])

        # BF_d2 =  self.BF.diff(self.x_o_s,2)
        # UnsafeInfo.CBF = lambdify([self.x_r_s,self.x_o_s], CBF)

    def details(self):
        return '{}\n {}\n {}\n'.format(self.h(*self.states), self.BF(*self.states), self.states)
    # def add_constraint(self):
    #     pass
    # def remove_constraint(self):
    #     pass

class Control_CBF(object):
    def __init__():
        self.CBF =
        self.ego = 
        self.agents = 
        self.P = 
        self.Q = 