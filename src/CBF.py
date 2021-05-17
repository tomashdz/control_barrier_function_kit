from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff
import numpy as np


class CBF(object):
    def __init__(self, h, h_inp ):
        self.hsym = h(*h_inp)


    def constraints(self):
        pass
    def compute_lambda_G(self):
        pass
    def compute_lambda_h(self):
        pass
    def add_constraint(self):
        pass
    def remove_constraint(self):
        pass
