from sympy import Matrix

class Controller(object):
    def __init__(self, C_list):
        self.C = Matrix(C_list)
        return
