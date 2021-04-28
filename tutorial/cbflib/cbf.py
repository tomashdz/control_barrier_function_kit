from sympy import symbols, lambdify, diff
import numpy as np


class CBF:
    def __init__(self, B, f, g, states, bad_sets, symbs):
        """ This initializes the CBF and computes functions for the G and h matrices for convex optimization later on.
        Args:
            B (sympy expression):   The expression for the bad set representation
            f (sympy expression):   The expression for the f(x) function
            g (sympy expression):   The expression for the g(x) function
            states (tuple):         A tuple with the states of the system
            bad_sets (list):        A list of bad sets, with each row consisting of z_0,z_1,a,b
                                    where z_0,z_1 represent the center and a,b represents the major,
                                    minor axes of the ellipse
        """
        self.B = B
        self.f = f
        self.g = g
        self.states = states
        self.bad_sets = bad_sets
        self.symbs = symbs
        self.G = []                 # G matrix for CVXopt
        self.h = []                 # h matrix for CVXopt
        self.expr_bs = []           # symbolic expressions for bad sets
        self.lamb_G = []            # function for computation of symbolic expression for G matrix
        for i in self.states:
            temp_expr = diff(B, i)
            self.expr_bs.append(temp_expr)
            self.lamb_G.append(
                lambdify([symbs], temp_expr, "math"))
        # function for computation of symbolic expression for h matrix
        self.lamb_h = lambdify([symbs], B, "math")

    def compute_G_h(self, x):
        """ The method computes the G and h matrices for convex optimization given current state

        Args:
            x (numpy.ndarray): array with the current state of the system

        Returns:
            list: returns G matrix
            list: returns h matrix
        """
        self.G = []
        self.h = []

        # for each bad set, given current state, compute the G and h matrices
        for idxi, _ in enumerate(self.bad_sets):
            curr_bs = self.bad_sets[idxi]
            tmp_g = []
            self.G.append([])
            for lamb in self.lamb_G:
                # tmp_g = lamb(tuple(np.hstack((x, curr_bs))))
                tmp_g = lamb(tuple(np.hstack((x, curr_bs[2:4]))))
                self.G[idxi].append(-1*tmp_g)
            self.h.append(self.lamb_h(tuple(np.hstack((x, curr_bs[2:4])))))
            # self.h.append(self.lamb_h(tuple(np.hstack((x, curr_bs)))))
        return self.G, self.h
