from sympy import symbols, lambdify, diff
from system import *

xr_0, xr_1, xr_3 = symbols('xr_0 xr_1 xr_2')
ur_0, ur_1 = symbols('ur_0 ur_1')
xo_0, xo_1 = symbols('xo_0 xo_1')

ego = System('ego','ego', Matrix([xr_0, xr_1, xr_3]), Matrix([ur_0, ur_1]))
# agent1 = System('agent1','agent',Matrix([xo_0, xo_1]))

print(ego.system_details())
# print(agent1.system_details())

# xr_2 = symbols('xr_2')
# ego.add_state((xr_2,))
# print(ego.system_details())

# System.set_solver('z')
# print(ego.system_details())
# print(agent1.system_details())

# print(System.add_numbers([1,2]))
ego_model = Model('ego', 'ego', Matrix([xr_0, xr_1, xr_3]),  Matrix([ur_0, ur_1]),'appr_unicycle', l = 0.1)
print(ego_model.system_details())


# agent_2_deter = Deterministic('agent2_deter',(xo_0, xo_1), [1, 1], [2, 2])
# print(agent_2_deter.system_details())

