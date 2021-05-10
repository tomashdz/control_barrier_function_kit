from sympy import symbols, lambdify, diff
from system import System

xr_0, xr_1 = symbols('xr_0 xr_1')
states=(xr_0, xr_1)

sys1 = System(states)
print(sys1.states)