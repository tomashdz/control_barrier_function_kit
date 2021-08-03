from sympy import symbols
import numpy as np
import matplotlib.pyplot as plt
import control as control
import cvxopt as cvxopt
from cbflib import cbf, cbf_utils, sys_and_ctrl
import matplotlib.animation as animation
from matplotlib.patches import Ellipse
from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff, Mul, srepr

# Robot Goal
x_goal = np.array([10, 3.5])

# The bad set defines the radii of the eclipse. The position is part of the dynamics of the system.
bad_sets = [[3, 2]]

# Parameters for reference controller
ctrl_param = [0.3, 0.3]

# Symbols and equations for the CBF
xr_0, xr_1, xo_0, xo_1, a, b, xr_0_dot, xr_1_dot = symbols('xr_0 xr_1 xo_0 xo_1 a b xr_0_dot xr_1_dot')
symbs = (xr_0, xr_1, xo_0, xo_1, a, b)

# Barrier function - distance of robot to obstacle
B = ((xr_0 - xo_0)/a)**2 + ((xr_1 - xo_1)/b)**2 - 1

# dx = f(x) + g(x)u
f = Matrix([0, 0, -1.0, 0])
g = Matrix([xr_0, xr_1, 0, 0])
degree = 1

# Initialize CBF
my_CBF = cbf.CBF(B=B, f=f, g=g, states=(
    xr_0, xr_1, xo_0, xo_1), bad_sets=bad_sets, symbs=symbs, degree=1)

# Simulation settings
T_max = 20
n_samples = 100
T = np.linspace(0, T_max, n_samples)
dt = T[1]-T[0]
params = {'x_goal': x_goal, 'bad_sets': bad_sets,
          'ctrl_param': ctrl_param, 'CBF': my_CBF}

# intial condition
x_0 = np.array([0, 3.2, 10, 3.5])

# Disable cvxopt optimiztaion output
cvxopt.solvers.options['show_progress'] = False
cvxopt.solvers.options['max_iter'] = 1000

# Init Plot
fig, ax = plt.subplots()

# Simulate system
print('\nComputing trajectories for the initial condition:')
print('x_0\t x_1')
print(x_0[0], '\t', x_0[1])

# Compute output on the nimble ant system for given initial conditions and timesteps T
x = np.zeros((np.size(x_0), len(T)))
x[:, 0] = x_0

# Simulate system
for i in range(len(T)-1):
    x[:, i+1] = x[:, i] + dt * \
        np.array(sys_and_ctrl.nimble_ant_with_agent_f(
            T[i], x[:, i], [], params))

print("\n*Simulation Done. See plot for animation...")

# Animate
plt.xlim(0, 12)
plt.ylim(0, 6)
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)
line1, = ax.plot([], [], lw=2)
line2, = ax.plot([], [], lw=2)

ell = Ellipse((x[2][i-1], x[3][i-1]),
                  bad_sets[0][0], bad_sets[0][1], color='g', alpha=0.3)

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    ell.center = (x[2][0], x[3][0])
    return line1, line2


def animate(i):
    line1.set_data((x[0][0:i], x[1][0:i]))
    line2.set_data((x[2][0:i], x[3][0:i]))
    ell.center = (x[2][i], x[3][i])
    return line1, line2, ax.add_patch(ell)


ani = animation.FuncAnimation(
    fig, animate, init_func=init, interval=50, blit=False, frames=n_samples)

plt.show()

print("\n*Animation Complete. Exiting...\n")
