import numpy as np
import scipy.signal as signal
import scipy.integrate as spi
import matplotlib.pyplot as plt
import control as control
import cvxopt as cvxopt
from picos import Problem, RealVariable, BinaryVariable, Solution

def silly_bug_c(x,goal_x):
    # Control function 
    
    # Control parameters
    k1 = 0.2
    k2 = 0.2
 
    # Reference controller
    uref_1 = -k1 * (x[0] - goal_x[0]) - 1 
    uref_2 = -k2 * (x[1] - goal_x[1]) - 1

    # Init optimization 
    u = RealVariable("u", (2,1))        # A column vector with 2 elements.
    s = RealVariable("s")               # A column vector with 2 elements.
    P = Problem()                       # Initilize picos problem

    # Optimization problem
    P.set_objective("min", abs(u-[[uref_1],[uref_2]]-s))

    # Parameters for the CBF 
    g1 = 2*x[0]**2-6*x[0]
    g2 = 2*x[1]**2-6*x[1]
    g3 = (x[0] - 3)**2 + (x[1] - 3)**2 - 1

    # Constraints
    Const1 = P.add_constraint( [g1,0]*u +g1 + [0,g2]*u +g2 >= -1*((g3)-s))
    Const2 = P.add_constraint(s >= 0)

    # Options
    P.options.solver = "cvxopt"
    P.options.primals = True

    # Solve and return
    solution = P.solve()
    sol_list = solution.primals
    sol = list(sol_list.values())
    return sol[1]

def silly_bug_f(t,x,u,params):
    # a function for a silly bug
    goal_x = [5,5]      # Robot goal
    xy = [x[0], x[1]]
    
    # compute control given current position
    u_0 = silly_bug_c(xy,goal_x)

    # compute change in xy direction 
    dx0 = 1 + u_0[0]
    dx1 = 1 + u_0[1]

    return [dx0, dx1]

# System definition using the control toolbox
silly_bug_sys = control.NonlinearIOSystem(
    silly_bug_f, None, inputs=None, outputs=None,
    states=('x0', 'x1'), name='silly_Bug')

# Simulation settings
T_max = 20       
n_samples = 50 
T = np.linspace(0, T_max, n_samples)

# Initial conditiosn
min_x, min_y, max_x, max_y = 0, 0, 4, 4     # min, max of x,y values for initial conditions
nx, ny = 4, 4               # number of initial conditions in each axis

# Vectors of initial conditions in each axis
xx = np.linspace(min_x, max_x, nx)
yy = np.linspace(min_y, max_y, ny)

# Plot
fig, ax = plt.subplots()

# Plot colors
jet = plt.get_cmap('jet')
colors = iter(jet(np.linspace(0,1,len(xx)*len(yy))))

# Loop through initial conditions
for idxi, i in enumerate(xx):
    for idxk, k in enumerate(yy):
        # If initial condition is inside the bad set, skip it. 
        if ((i-3)**2+(k-3)**2<=1):
            print('Skipped: ',i,k, ' inside bad set')
            continue

        print('Computing: ',i,k)

        x_0 = np.array([i,k])
        
        # Compute output on the silly bug system for given initial conditions and timesteps T
        t, y, x = control.input_output_response(sys=silly_bug_sys, T=T, U=0, X0=[i,k], return_x=True)

        # Plot initial conditions and path of system
        plt.plot(i,k, 'x-', markersize=5, color=[0,0,0,1])
        plt.plot(x[0], x[1], 'o-', markersize=1, color=next(colors,[1,1,1,1]))
        
# Plot bad set
circle1 = plt.Circle((3, 3), 1, color='r')
ax.add_patch(circle1)
plt.xlim(0, 6)
plt.ylim(0, 6)
plt.show()  