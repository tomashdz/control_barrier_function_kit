from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff
import numpy as np

class System(object):
    # Includes an object that models ego system and agents
    # solver = 'ode45'
    def __init__(self, name, sys_type, states, inputs):
        self.name = name        # Do we need name??
        self.type = sys_type    # agent or ego:  Describes whether we have control or not
        self.states = states
        self.inputs = inputs
        self.d_states = np.zeros([len(states),1])  # Initiated as an static object

    # @property
    def system_details(self):
        return '{} {} {}'.format(self.name, self.type, self.states, self.inputs)

    # def add_state(self, new_state):
    #     self.states += new_state

    # @classmethod
    # def set_solver(cls, solver):
    #     cls.solver = solver

    # @staticmethod
    # def add_numbers(num_list):
    #     return sum(num_list)




class Model(System):
    def __init__(self,name, sys_type, states, inputs, modelname,**kwargs):
        super(Model, self).__init__(name, sys_type, states, inputs)
        models = ["appr_unicycle", "nimble_ant"]
        self.info = type('',(),{})()
        if modelname in models:
            self.modelname = modelname
            self.args = kwargs
            self.info.f, self.info.g, self.info.dx = self.modelinfo()
        else:
            print("Cannot find this model name. Please define it manually.")


    def  modelinfo(self):
        if self.modelname == "appr_unicycle":
            if len(self.states) != 3 or len(self.inputs)!=2:
                 raise ValueError("appr_unicycle model has 3 states and 2 inputs")
            for key, value in self.args.items():
                if key == "l":
                    l = value
            try: l
            except NameError: ValueError('you need to define l for this model')
            else:
                f = Matrix([0,0,0])
                g = Matrix([[cos(self.states[2]), -l*sin(self.states[2])], [sin(self.states[2]), l*cos(self.states[2])], [0, 1]])
                dx = f+g*self.inputs
        elif self.modelname == "nimble_ant":
            if len(self.states) != 2 or len(self.inputs) != 2: raise ValueError("nimble_ant model has 2 states")
            f = Matrix([0,0])
            g = Matrix([1,1])
            dx = f+g*self.inputs
        else:
            raise ValueError("model not defined")
        return f,g,dx
    def system_details(self):
            return '{} {} {} {} {}'.format(self.name, self.type, self.states, self.modelname, self.info.__dict__, self.args)



class Deterministic(Model):
    def __init__(self, name, sys_type, states, inputs, modelname,**kwargs):
        super(Deterministic, self).__init__(self,name, sys_type, states, inputs, modelname,**kwargs)      


    def system_details(self):
        return '{} {} {} {} {}'.format(self.name, self.states, self.type, self.C, self.D)
    def x_dot(self):
        x_dot = f
        return x_dot


class Stochastic(Model):
    def __init__(self, name, sys_type, states,inputs, *args):
        super(Stochastic, self).__init__(self,name, sys_type, states, inputs, modelname,**kwargs)
        self.C = C
        self.D = D
    def system_details(self):
        return '{} {} {} {} {}'.format(self.name, self.states, self.type, self.C, self.D)
    def dx():
        x_dot = f
        return x_dot