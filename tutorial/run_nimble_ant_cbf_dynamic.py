from sympy import symbols
import numpy as np
import matplotlib.pyplot as plt
import control as control
import cvxopt as cvxopt
from cbflib import cbf, cbf_utils, sys_and_ctrl
import matplotlib.animation as animation
from matplotlib.patches import Ellipse

# Robot Goal
x_goal = np.array([10, 3.5])

# Elipse format (x,y,rad_x,rad_y)
bad_sets = cbf_utils.example(4)

# Parameters for reference controller
ctrl_param_k = [0.3, 0.3]

# Symbols and equations for the CBF
xr_0, xr_1, xo_0, xo_1, a, b = symbols('xr_0 xr_1 xo_0 xo_1 a b')
symbs = (xr_0, xr_1, xo_0, xo_1, a, b)
B = ((xr_0 - xo_0)/a)**2 + ((xr_1 - xo_1)/b)**2 - 1

# ! placeholders for more complex functions
f, g = 1, 1

# Initialize CBF
# my_CBF = cbf.CBF(B=B, f=f, g=g, states=(xr_0, xr_1), agent_states=(xo_0, xo_1), bad_sets=bad_sets, symbs=symbs)
my_CBF = cbf.CBF(B=B, f=f, g=g, states=(
    xr_0, xr_1, xo_0, xo_1), bad_sets=bad_sets, symbs=symbs)

# Simulation settings
T_max = 20
n_samples = 200
T = np.linspace(0, T_max, n_samples)

# System definition using the control toolbox
nimble_ant_with_agent_sys = control.NonlinearIOSystem(
    sys_and_ctrl.nimble_ant_with_agent_f, None , inputs=('u_0', 'u_1'), outputs=('xr_0', 'xr_1', 'xo_0', 'xo_1'), dt=None,
    states=('xr_0', 'xr_1', 'xo_0', 'xo_1'), name='nimble_ant_with_agent_sys')

nimble_ant_with_agent_ctrl = control.NonlinearIOSystem(
    None, sys_and_ctrl.nimble_ant_with_agent_c, inputs=('xr_0', 'xr_1', 'xo_0', 'xo_1'), outputs=('u_0', 'u_1'), dt=None,
    name='nimble_ant_with_agent_ctrl', params={'x_goal': x_goal, 'bad_sets': bad_sets, 'ctrl_param_k': ctrl_param_k, 'CBF': my_CBF})

nimble_ant_closed = control.InterconnectedSystem(
    (nimble_ant_with_agent_sys, nimble_ant_with_agent_ctrl),       # systems
    connections=(
        ('nimble_ant_with_agent_sys.u_0', 'nimble_ant_with_agent_ctrl.u_0'),
        ('nimble_ant_with_agent_sys.u_1', 'nimble_ant_with_agent_ctrl.u_1'),
        ('nimble_ant_with_agent_ctrl.xr_0', 'nimble_ant_with_agent_sys.xr_0'),
        ('nimble_ant_with_agent_ctrl.xr_1', 'nimble_ant_with_agent_sys.xr_1'),
        ('nimble_ant_with_agent_ctrl.xo_0', 'nimble_ant_with_agent_sys.xo_0'),
        ('nimble_ant_with_agent_ctrl.xo_1', 'nimble_ant_with_agent_sys.xo_1')
    ),
    #   inplist=None,
    outlist=('nimble_ant_with_agent_sys.xr_0', 'nimble_ant_with_agent_sys.xr_1',
             'nimble_ant_with_agent_sys.xo_0', 'nimble_ant_with_agent_sys.xo_1'),
    states=('xr_0', 'xr_1', 'xo_0', 'xo_1')
)

# Initial conditions
# min, max of x,y values for initial conditions
min_x, min_y, max_x, max_y = 0, 0, 0.5, 0.5

# number of initial conditions in each axis
nx, ny = 3, 3

# Vectors of initial conditions in each axis
xx = np.linspace(min_x, max_x, nx)
yy = np.linspace(min_y, max_y, ny)

# Uncomment the following for specific intial conditions
xx = [0]
yy = [3.2]

# Disable cvxopt optimiztaion output
cvxopt.solvers.options['show_progress'] = False
cvxopt.solvers.options['max_iter'] = 1000

# Plot
fig, ax = plt.subplots()
jet = plt.get_cmap('jet')
colors = iter(jet(np.linspace(0, 1, len(xx)*len(yy))))

# Loop through initial conditions and simulate system
print('Computing trajectories for initial conditions:')
print('x_0\t x_1')
for idxi, i in enumerate(xx):
    for idxj, j in enumerate(yy):
        # If initial condition is inside the bad set, skip it.
        bool_val = 0
        curr_bs = []
        for idxk, k in enumerate(bad_sets):
            curr_bs = bad_sets[idxk]
            if cbf_utils.is_inside_ellipse([i, j], bad_sets[idxk]):
                print('Skip (Invalid):\t', i, j)
                bool_val = 1
        if bool_val == 1:
            continue

        print(round(i, 2), '\t', round(j, 2), "\t... ", end="", flush=True)
        xr_0 = np.array([i, j])

        # Compute output on the nimble ant system for given initial conditions and timesteps T
        t, y, x = control.input_output_response(sys=nimble_ant_closed, T=T, U=0, X0=[
                                                i, j, 10, 3.5], return_x=True, method='BDF')

        # # Plot initial conditions and path of system
        # plt.plot(i, j, 'x-', markersize=3, color=[0, 0, 0, 1])
        # plt.plot(x[0], x[1], 'o-', markersize=1,
        #          color=next(colors, [1, 1, 1, 1]))

        # plt.plot(x[2], x[3], 'x-', markersize=1,
        #          color=next(colors, [0, 0, 1, 1]))

        print("Done")

# ax = cbf_utils.plot_cbf_elements(ax, bad_sets, x_goal)

plt.xlim(0, 12)
plt.ylim(0, 6)
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)
line1, = ax.plot([], [], lw=2)
line2, = ax.plot([], [], lw=2)

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    ims = []
    goal_square = plt.Rectangle(
        x_goal-np.array([.1, .1]), .2, .2, color='g', alpha=0.5)
    ims.append(goal_square)
    return ims

def animate(i):
    ims = []
    line1.set_data((x[0][0:i], x[1][0:i]))
    line2.set_data((x[2][0:i], x[3][0:i]))
    ell = Ellipse((x[2][i-1], x[3][i-1]), 
                      1, 1, color='g', alpha=0.3)
    patch = ax.add_patch(ell)
    ims.append(line1)
    ims.append(line2)
    ims.append(patch)
    return ims

ani = animation.FuncAnimation(
    fig, animate,init_func=init, interval=50, blit=True, frames=n_samples)

plt.show()