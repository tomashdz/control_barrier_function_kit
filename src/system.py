from sympy import symbols, lambdify, diff
import numpy as np

class System(object):

    solver = 'ode45'

    def __init__(self, name, states):
        self.name = name
        self.states =  states

    # @property
    def system_details(self):
        return '{} {} {}'.format(self.name, self.states, self.solver)

    def add_state(self, new_state):
        self.states += new_state

    @classmethod
    def set_solver(cls, solver):
        cls.solver = solver

    @staticmethod
    def add_numbers(num_list):
        return sum(num_list)

class Deterministic(System):
    def __init__(self, name, states, C, D):
        super().__init__(name, states)
        self.C = C
        self.D = D
    def system_details(self):
        return '{} {} {} {} {}'.format(self.name, self.states, self.solver, self.C, self.D)