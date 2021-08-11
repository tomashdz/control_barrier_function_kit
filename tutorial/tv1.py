from sympy import symbols
import numpy as np
import matplotlib.pyplot as plt
import control as control
import cvxopt as cvxopt
from cbflib import cbf, cbf_utils, sys_and_ctrl
import matplotlib.animation as animation
from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff, Mul, srepr

# Robot Goal
x_goal = np.array([8, 8])

# Undesired areas in ellipse format (x,y,rad_x,rad_y) - Use example(0) through example(3)
bad_sets = [] #cbf_utils.example(1)

# Parameters for reference controller
ctrl_param = []

# Symbols and equations for the CBF
# x_0, x_1, z_0, z_1, a, b, t = symbols('x_0 x_1 z_0 z_1 a b t')
xr0, xr1, u, t, xr0_dot, xr1_dot, = symbols(
    'xr0 xr1 u t xr0_dot xr1_dot')
# Barrier function - distance of robot to obstacle
# B = ((x_0 - z_0)/a)**2 + ((x_1 - z_1)/b)**2 - 1


# B = -1*t + (sum(x_goal**2)**.5) - sqrt((xr0-x_goal[0])**2 + (xr1-x_goal[1])**2)

#! F_[5,15](||robot - goal||< 1) -> F_[0,15](||robot - goal||< 1)

#! F_[0,15](||robot - goal||< 1)
B = -1*t + 16 - sqrt((xr0-x_goal[0])**2 + (xr1-x_goal[1])**2)

# ! G_[5,15] (goal<value)
B = 11*exp(-0.4796*t) + 9 - sqrt((xr0-x_goal[0])**2 + (xr1-x_goal[1])**2)

# -1*t + 5 - sqrt((xr0-x_goal[0])**2 + (xr1-x_goal[1])**2)
# B = -5/15*t + (sum(x_goal**2)) - sqrt((xr0-x_goal[0])**2 + (xr1-x_goal[1])**2)


# dx = g(x)u - not used
f = 0
g = Matrix([0, 0, 1.0])

# Initialize CBF
my_CBF = cbf.CBF(B, f, g, states=(xr0,xr1), states_dot=[xr0_dot,xr1_dot], symbs=(xr0,xr1), degree=1, time_varying = 1)

# Simulation settings
T_max = 15
n_samples = 200
T = np.linspace(0, T_max, n_samples)
dt = T[1]-T[0]
params = {'x_goal': x_goal, 'bad_sets': bad_sets,
          'ctrl_param': ctrl_param, 'CBF': my_CBF}

# Disable cvxopt optimiztaion output
cvxopt.solvers.options['show_progress'] = False

# intial condition
x_0 = np.array([0, 0])

# Simulate system
print('\nComputing trajectories for the initial condition:')
print('x_0\t x_1')
print(x_0[0], '\t', x_0[1])

# If initial condition is inside the bad set, error out.
for idxj, j in enumerate(bad_sets):
    curr_bs = bad_sets[idxj]
    assert cbf_utils.is_inside_ellipse(
        [x_0[0], x_0[1]], bad_sets[idxj]) == 0, "Initial condition is inside ellipse"

# Compute output on the nimble ant system for given initial conditions and timesteps T
x = np.zeros((np.size(x_0), len(T)))
x[:, 0] = x_0

# Simulate system
for i in range(len(T)-1):
    x[:, i+1] = x[:, i] + dt * \
        np.array(sys_and_ctrl.nimble_ant_tv_f(T[i], x[:, i], [], params))

print("\n*Simulation Done. See plot for animation...")

# Init Plot
fig, ax = plt.subplots()

# Animate
ax = cbf_utils.plot_cbf_elements(ax, bad_sets, x_goal)

plt.xlim(-2, 12)
plt.ylim(-2, 12)
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)

line1, = ax.plot([], [], lw=2)
goal_square = plt.Rectangle(
    x_goal-np.array([1, 1]), 1, 1, color='g', alpha=0.5)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

def init():
    line1.set_data([], [])
    return line1


def animate(i):
    line1.set_data((x[0][0:i], x[1][0:i]))
    time_text.set_text(i*dt)
    return line1, time_text,


ani = animation.FuncAnimation(
    fig, animate, init_func=init, interval=10, frames=n_samples, repeat=False)

plt.show()
print(x[0][-1], x[1][-1])
print((((x[0][-1]-x_goal[0])**2 + (x[1][-1]-x_goal[1])**2)**0.5))

print("\n*Animation Complete. Exiting...\n")
