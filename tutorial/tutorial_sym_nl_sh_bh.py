import numpy as np
import matplotlib.pyplot as plt
import cvxopt as cvxopt
from matplotlib.patches import Ellipse
from numpy import linalg
from numpy.lib.function_base import append
import numpy.random as rnd
from sympy import symbols, Symbol,Function, Matrix, sin, cos, lambdify, exp, sqrt, log, diff, Mul, srepr
from sympy.diffgeom import LieDerivative
from sympy.diffgeom.rn import R2_r
import math
import random
import matplotlib.animation as animation
from matplotlib.patches import Ellipse
from sympy.solvers.diophantine.diophantine import length

def n_omni_robots_c(x, params):
    # Controller for nimble car
    x_goal = params['x_goal']
    ctrl_param = params['ctrl_param']
    myCBF = params['myCBF']

    # Reference controller
    # theta_ref = math.atan((goal_x[1]-x[1])/(goal_x[0]-x[0]))
    # uref_0 = ctrl_param[0] * (theta_ref - x[2])

    # math.atan2(sin(theta_ref-x[2]), cos(theta_ref-x[2]))

    ############################
    # cvxopt quadratic program
    # minimize  0.5 x'Px + q'x
    # s.t       Gx<=h
    ############################
    # P matrix
    P = cvxopt.matrix(np.eye(4))
    # P = .5 * (P + P.T)  # symmetric

    # q matrix
    q = 0 #cvxopt.matrix(np.array([-1*uref_0]), (1, 1))

    # F_[5,15](||x-x_g||)<=5)
    # h(x) = 5 - ||x-x_g||
    # b(x,t) = y(t) -  ||x-x_g||
    # y(t) = -5/15*t + 10

    G, h = myCBF.compute_G_h(x)

    G = cvxopt.matrix(G)
    h = cvxopt.matrix(h)

    # Run optimizer and return solution
    # sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
    try:
        sol = cvxopt.solvers.qp(P, q, G.T, h, None, None)
        x_sol = sol['x']
    except:
        x_sol = [0]
        print("bad")
    # print(x, ' G: ', G, ' h: ', h, ' x_sol: ', x_sol)
    return x_sol[0:1]

def n_omni_robots_f(t, x, u, params):

    # f = np.zeros((3, 3))
    f = []
    sum_1 = 0
    sum_2 = 0
    # k = 0.025

    for i in range(0, x.shape[0]):
        for j in range(0, x.shape[0]):
            if i != j:
                # define control parameter k for omnidirectional robot
                k = max( (0.5 - (np.linalg.norm(x[i,0:2] - x[j,0:2])) )/20,0.0001)
                # k = 0.025
                # print('k ', k, 'norm', np.linalg.norm(x[i,0:2] - x[j,0:2]))
                sum_1 = sum_1 + k * (x[i, 0] - x[j, 0]) / \
                    (np.linalg.norm(x[i,0:2] - x[j,0:2]) + 0.000001)
                # print(x[i, 0] - x[j, 0],np.linalg.norm(x[i,0:2] - x[j,0:2]) + 0.000001, sum_1)
                sum_2 = sum_2 + k * (x[i, 1] - x[j, 1]) / \
                    (np.linalg.norm(x[i,0:2] - x[j,0:2]) + 0.000001)
        f.append([])
        f[i].append(sum_1)
        f[i].append(sum_2)
        f[i].append(0)
        sum_1 = 0
        sum_2 = 0

    f = np.array(f)
    print(f)
    # dynamics of the robot
    R = 0.02
    L = 0.2
    newg = np.zeros((x.shape[0],3,3), dtype=float)
    for i in range(0, x.shape[0]):
        # np.append(g,[])
        newg[i] = np.array([[[cos(x[i][2]), -1*sin(x[i][2]), 0],
                              [sin(x[i][2]), cos(x[i][2]), 0], [0, 0, 1]]])

    B = np.array([[0, cos(math.pi/6), -1*cos(math.pi/6)],
                  [-1, sin(math.pi/6), sin(math.pi/6)],
                  [L, L, L]], dtype=float)

    u = np.zeros((3,1))
    u[0] = 0
    u[1] = -1
    u[2] = 1

    dx = np.zeros((x_0.shape[0],x_0.shape[1]))

    for i in range(0, x.shape[0]):
        if i == 0:
            u[0] = 0
            u[1] = 1
            u[2] = -1
        if i == 1:
            u[0] = 0
            u[1] = -1
            u[2] = 1
        temp = np.matmul(newg[i],np.linalg.inv(B.T)) * R
        temp2 = np.matmul(temp,u)
        dx[i] =  f[i] + temp2.T

    return dx


def example(i):
    # Examples of different bad sets
    # The x,y,z,d where x,y represent the center and z,d represents the major, minor axes of the ellipse
    switcher = {
        0: [[3, 2., 1., 1.]],
        1: [[1., 2., 0.5, 0.5], [4., 1., 0.5, 0.5],
            [3., 2., 0.5, 0.5], [4.5, 4.2, 0.5, 0.5]],
        2: [[3.5, 1., 0.2, 2.], [2., 2.5, 1., 0.2], [1.5, 1., 0.5, 0.5]],
        3: [[3.5, 3., 0.2, 2.], [2., 2.5, 1., 0.2], [1.5, 1., 0.5, 0.5]]
    }
    return switcher.get(i, "Invalid")


def is_inside_ellipse(x, x_e):
    if ((x[0] - x_e[0])/x_e[2])**2 + ((x[1] - x_e[1])/x_e[3])**2 <= 1:
        return 1
    else:
        return 0

x_goal = np.array([10, 0], dtype=float)
x_goal = np.tile(x_goal, (1, 1))


# Elipse format (x,y,rad_x,rad_y)
bad_sets = example(3)

# Parameters for reference controller
ctrl_param = [5]

xr0, xr1, xr2, u = symbols(
    'xr0 xr1 xr2 u')

# B = ((xr0 - cx)/rad_x)**2 + ((xr1 - cy)/rad_y)**2 - 1
# B = (xr0 - cx)**2 + (xr1 - cx)**2 - 1
f = Matrix([0, 0, 0])

R = 0.02
L = 0.2
Bg = np.array([[0, cos(math.pi/6), -1*cos(math.pi/6)],
    [-1, sin(math.pi/6), sin(math.pi/6)],
    [L, L, L]], dtype=float)
Bgg = Matrix(np.linalg.inv(Bg.T)*R)
Pg = Matrix([[cos(xr2), -1*sin(xr2), 0],
        [sin(xr2), cos(xr2), 0], [0, 0, 1.0]]) 
g = Pg * Bgg


states = Matrix([xr0, xr1, xr2])

# expr_bs_dx0 = diff(expr_bs,xr0)
# expr_bs_dx1 = diff(expr_bs,xr1)

t = Symbol('t')
B = Function('B')(t)
B = -5/15*t + 10 - (Matrix([xr0, xr1]) - Matrix(x_goal[0])).norm()
B.subs(t,2)
B_dot = []
for i in states:
    B_dot.append(diff(B, i))


# ? Simulation settings
T_max = 200
n_samples = 100
T = np.linspace(0, T_max, n_samples)
dt = T[1]-T[0]
params = {'x_goal': x_goal, 'bad_sets': bad_sets,
          'ctrl_param': ctrl_param,}

# System definition using the control toolbox
# nimble_car_sys = control.NonlinearIOSystem(
#     nimble_car_f, None, inputs=None, outputs=None, dt=None,
#     states=('x0', 'x1', 'x2'), name='nimble_car',
#  params=params)

# ? Initial conditions
# ? min, max of x,y values for initial conditions
min_x, min_y, max_x, max_y = -0.5, -0.5, 0.5, 0.5
nx, ny = 10, 10              # number of initial conditions in each axis

# Vectors of initial conditions in each axis
xx = np.linspace(min_x, max_x, nx)
yy = np.linspace(min_y, max_y, ny)

# ? Uncomment the following for specific intial conditions
xx = [0.5]
yy = [0.5]


# exmample 1
# x1 = np.array([1, 2, 0], dtype=np.fl oat32)
# x2 = np.array([4, 2.1, 0])
# x_0 = np.vstack([x1, x2])

# exmample 2
x1 = np.array([1, 1, math.pi/4], dtype=np.float32)
x2 = np.array([1, 4, 3*math.pi/4])
x3 = np.array([3, 1, 7*math.pi/4])
x4 = np.array([2, 4, math.pi/4])
# x_0 = np.vstack([x1, x2, x3, x4])
x_0 = np.vstack([x1])

for idxi, i in enumerate(xx):
    for idxk, k in enumerate(yy):

        print(round(i, 2), '\t', round(k, 2), "\t... ", end="", flush=True)

        # Compute output on the silly bug system for given initial conditions and timesteps T
        x = np.zeros((len(T),x_0.shape[0],x_0.shape[1]), dtype=np.float64)
        x[0] = x_0
        for i in range(len(T)-1):
            x[i+1] = x[i] + dt * \
                np.array(n_omni_robots_f(T[i], x[i], [], params)) 
            # B = -5/15*i*dt + 10 - linalg.norm(x[0][:,0:2] - x_goal)
        colors = ["red", "green", "purple"]

# Plot
fig, ax = plt.subplots()
jet = plt.get_cmap('jet')

plt.xlim(0, 5)
plt.ylim(0, 5)
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)

line1, = ax.plot([], [], lw=2)
line2, = ax.plot([], [], lw=2)
line3, = ax.plot([], [], lw=2)
line4, = ax.plot([], [], lw=2)

# lines = [line1, line2, line3, line4]
lines = [line1]

x1 = np.array([1, 1, math.pi/4], dtype=np.float32)
x2 = np.array([1, 4, 3*math.pi/4])
x3 = np.array([3, 1, 7*math.pi/4])
x4 = np.array([2, 4, math.pi/4])
x_0 = np.vstack([x1, x2, x3, x4])

for i in range(len(x_0)):
    ell = Ellipse((x_0[i][0], x_0[i][1]), width=0.1, height=0.1, color='g', alpha=0.5)
    ax.add_artist(ell)

# animate path of two robots
def animate(i):
    ims = []
    #iterate over lines
    for j in range(len(lines)):
        print(i,j)
        lines[j].set_data((x[0:i,j,0], x[0:i,j,1]))
        ims.append(lines[j])

    # if i % 10 == 0:
    #     print('hi')

    return ims

# n_samples = 98

ani = animation.FuncAnimation(
    fig, animate, interval=50, blit=True, frames=n_samples)

plt.show()