import numpy as np
import scipy.signal as signal
# import scipy.integrate as spi
import matplotlib.pyplot as plt
import control as control
import cvxopt as cvxopt
from cvxopt import solvers, matrix
from picos import Problem, RealVariable, BinaryVariable, Solution, Constant
from cvxopt.modeling import variable, op, max, sum
import pylab

def silly_bug_c(x,params):
    # Control function 
    goal_x = params['goal_x']
    bad_sets = params['bad_sets']

    # Control parameters
    k1 = 0.2
    k2 = 0.2
 
    # Reference controller
    uref_1 = k1 * ((goal_x[0]-x[0]))
    uref_2 = k2 * ((goal_x[1]-x[1]))

    # x_sol = [[0],[0]]

    # A = matrix([[0,0,1,0,0],[0,0,0,1,0.]]) 
    
    # x = [u_1,u_2,uref_1,uref_2.,s]
    # A = A.T

    # b = matrix([[uref_1],[uref_2]],(2,1))
    
    # (x0 - u0)^2 + (x1-u1)^2 = x0^2 - 2x0u0 + u0^2 ... 

    
    P = matrix(np.eye(3))
    P[2,2] = 0
    P = .5 * (P + P.T)
    q = matrix(np.array([-uref_1, -uref_2, 1]),(3,1))

    # G = matrix([0,0,0])
    # G = G.T
        
    # h = matrix([0])

    # Parameters for the CBF 
    curr_bs = []
    for idxi, _ in enumerate(bad_sets):
        curr_bs = bad_sets[idxi]
        g1 = 2*x[0]**2 - 2*curr_bs[0]*x[0]
        g2 = 2*x[1]**2 - 2*curr_bs[1]*x[1]
        g3 = (x[0] - curr_bs[0])**2 + (x[1] - curr_bs[1])**2 - curr_bs[2]  
                
        # G = matrix([[-g1,-g2,0,0,1.], [0,0,0,0,-1.] ])
        
        # ([g1,0]*u + [0,g2]*u >= -1*(g3+s)) the more negative s is, the safer the system is 
        
        G = matrix([[-g1,-g2,-1.0]])
        
        # new_G_const = matrix([[-g1,-g2,-1.]])
        # G = matrix([[G,new_G_const.T]])

        h = matrix([[g3]])

        # new_h_const = matrix([[g3]])
        # h = matrix([ [h,new_h_const] ])
        
        if params['verbose'] == 1:
            print('P = ', P)
            print('q = ', q)
            print('G = ', G)
            print('h = ', h)
            print('A = ', A)
            print('b = ', b)

        solvers.options['show_progress'] = False
        # solvers.options['max_iter'] = 100
        # solvers.options['abstol'] = 0.01
        # solvers.options['reltol'] = 0.01
        
    #return  
    sol = cvxopt.solvers.qp(P,q,G.T,h,None,None)
    x_sol = sol['x']  
    # print('x:',x_sol[0:2])
    # print('***************')
    return x_sol[0:2] #- np.array([[1],[1]])
    

def silly_bug_f(t,x,u,params):
    # a function for a silly bug
        
    # compute control given current position
    u_0 = silly_bug_c(x,params)

    # compute change in xy direction 
    dx0 = u_0[0]
    dx1 = u_0[1]

    return [dx0, dx1]

# Robot Goal
goal_x = np.array([5,5])
bad_sets = [[3.,2.,1]]
# bad_sets = [[1.,2.,0.5],[4,1,0.5]]
verbose = 0

# Simulation settings
T_max = 20
n_samples = 100
T = np.linspace(0, T_max, n_samples)

# Initial conditiosn
min_x, min_y, max_x, max_y = 0, 0, 4, 4     # min, max of x,y values for initial conditions
nx, ny = 5, 5               # number of initial conditions in each axis

# Vectors of initial conditions in each axis
xx = np.linspace(min_x, max_x, nx)
yy = np.linspace(min_y, max_y, ny)

xx = [0]
yy = [1]

# xx = [3.2]
# yy = [2.2]

# System definition using the control toolbox
silly_bug_sys = control.NonlinearIOSystem(
    silly_bug_f, None, inputs=None, outputs=None, dt=None,
    states=('x0', 'x1'), name='silly_bug',params={'goal_x': goal_x, 'bad_sets': bad_sets, 'verbose': verbose})

# Plot
fig, ax = plt.subplots()

# Plot colors
jet = plt.get_cmap('jet')
colors = iter(jet(np.linspace(0,1,len(xx)*len(yy))))


# Loop through initial conditions
for idxi, i in enumerate(xx):
    for idxk, k in enumerate(yy):
        # If initial condition is inside the bad set, skip it. 
        bool_val = 0
        curr_bs = []
        for idxj, j in enumerate(bad_sets):
            curr_bs = bad_sets[idxj]
            if ((i-curr_bs[0])**2+(k-curr_bs[1])**2<=curr_bs[2]):
                print('Skipped (IBS):\t',i,k)
                bool_val = 1
        if bool_val == 1:
            continue

        print('Computing:\t',i,k)
        x_0 = np.array([i,k])
        
        # Compute output on the silly bug system for given initial conditions and timesteps T
        t, y, x = control.input_output_response(sys=silly_bug_sys, T=T, U=0, X0=[i,k], return_x=True, method='BDF')

        # Plot initial conditions and path of system
        plt.plot(i,k, 'x-', markersize=5, color=[0,0,0,1])
        plt.plot(x[0], x[1], 'o-', markersize=2, color=next(colors,[1,1,1,1]))

curr_bs = []
for idxi, i in enumerate(bad_sets):
    curr_bs = bad_sets[idxi]
    circle = plt.Circle((curr_bs[0], curr_bs[1]), curr_bs[2], color='r')
    ax.add_patch(circle)
    
# Plot bad set
# circle1 = plt.Circle((3, 3), 1, color='r')
# ax.add_patch(circle1)

# Plot goal set
goal_square = plt.Rectangle(goal_x-np.array([.1,.1]), .2, .2, color='g')
ax.add_patch(goal_square)

plt.xlim(0, 6)
plt.ylim(0, 6)
plt.show()  