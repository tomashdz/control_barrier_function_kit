from sympy import symbols, lambdify, diff
import numpy as np

class System(object):

    def __init__(self, states):
        self.states =  states