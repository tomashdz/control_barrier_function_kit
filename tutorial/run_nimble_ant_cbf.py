from sympy import symbols
import numpy as np
import matplotlib.pyplot as plt
import control as control
import cvxopt as cvxopt
from cbflib import cbf, cbf_utils, sys_and_ctrl

# Robot Goal
x_goal = np.array([5, 5])

# Elipse format (x,y,rad_x,rad_y)
bad_sets = cbf_utils.example(2)

# Parameters for reference controller
ctrl_param_k = [0.3, 0.3]

# Symbols and equations for the CBF
x_0, x_1, z_0, z_1, a, b = symbols('x_0 x_1 z_0 z_1 a b')
symbs = (x_0, x_1, z_0, z_1, a, b)
B = ((x_0 - z_0)/a)**2 + ((x_1 - z_1)/b)**2 - 1

# ! placeholders for more complex functions
f, g = 1, 1

# Initialize CBF
my_CBF = cbf.CBF(B, f, g, (x_0, x_1), bad_sets, symbs)

# Simulation settings
T_max = 20
n_samples = 100
T = np.linspace(0, T_max, n_samples)

# System definition using the control toolbox
nimble_ant_sys = control.NonlinearIOSystem(
    sys_and_ctrl.nimble_ant_f, None, inputs=('u_0', 'u_1'), outputs=('x_0', 'x_1'), dt=None,
    states=('x_0', 'x_1'), name='nimble_ant_sys')

nimble_ant_ctrl = control.NonlinearIOSystem(
    None, sys_and_ctrl.nimble_ant_c, inputs=('x_0', 'x_1'), outputs=('u_0', 'u_1'), dt=None, name='nimble_ant_ctrl',
    params={'x_goal': x_goal, 'bad_sets': bad_sets, 'ctrl_param_k': ctrl_param_k, 'CBF': my_CBF})

nimble_ant_closed = control.InterconnectedSystem(
    (nimble_ant_sys, nimble_ant_ctrl),       # systems
    connections=(
        ('nimble_ant_sys.u_0', 'nimble_ant_ctrl.u_0'),
        ('nimble_ant_sys.u_1', 'nimble_ant_ctrl.u_1'),
        ('nimble_ant_ctrl.x_0', 'nimble_ant_sys.x_0'),
        ('nimble_ant_ctrl.x_1', 'nimble_ant_sys.x_1')
    ),
    #   inplist=None,
    outlist=('nimble_ant_sys.x_0', 'nimble_ant_sys.x_1')
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
yy = [0]

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
        x_0 = np.array([i, j])

        # Compute output on the nimble ant system for given initial conditions and timesteps T
        t, y, x = control.input_output_response(sys=nimble_ant_closed, T=T, U=0, X0=[
                                                i, j], return_x=True, method='BDF')

        # Plot initial conditions and path of system
        plt.plot(i, j, 'x-', markersize=3, color=[0, 0, 0, 1])
        plt.plot(x[0], x[1], 'o-', markersize=1,
                 color=next(colors, [1, 1, 1, 1]))

        print("Done")

ax = cbf_utils.plot_cbf_elements(ax, bad_sets, x_goal)

plt.xlim(0, 6)
plt.ylim(0, 6)
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)

plt.show()
