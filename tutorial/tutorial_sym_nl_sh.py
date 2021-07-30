import numpy as np
import matplotlib.pyplot as plt
import cvxopt as cvxopt
from matplotlib.patches import Ellipse
import numpy.random as rnd
from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff, Mul, srepr, simplify
from sympy.diffgeom import LieDerivative
from sympy.diffgeom.rn import R2_r
import math
import matplotlib.animation as animation


class CBF:
    def __init__(self, B, f, g, states, bad_sets, states_dot):
        self.B = B
        self.phi = []
        self.states = states
        self.G = []
        self.h = []
        self.expr_bs = []
        self.lamb_G = []

        expr = self.get_expr(B, f, g, states, states_dot)

        G, h = self.decompose_G_h(expr, g, states_dot)
        self.lamb_G.append(
            lambdify([(cx, cy, rad_x, rad_y, xr0, xr1, xr2)], G, "math"))
#!  h+B is incorrect when second order system,
        self.lamb_h = lambdify(
            # [(cx, cy, rad_x, rad_y, xr0, xr1, xr2)], (h+B), "math")
            [(cx, cy, rad_x, rad_y, xr0, xr1, xr2)], h, "math")

    def compute_G_h(self, x):
        self.G = []
        self.h = []
        for idxi, _ in enumerate(bad_sets):
            curr_bs = bad_sets[idxi]
            tmp_g = []
            self.G.append([])
            for lamb in self.lamb_G:
                tmp_g = lamb(tuple(np.hstack((curr_bs, x))))
                self.G[idxi].append(tmp_g)
            self.h.append(self.lamb_h(tuple(np.hstack((curr_bs, x)))))
        return self.G, self.h

    def get_expr(self, B, f, g, states, states_dot):
        a = 10
        B_dot_var = []
        for i in states:
            B_dot_var.append(diff(B, i))
        B_dot = Matrix(B_dot_var)
        B_dot_f = B_dot.T * states_dot
        phi = B_dot_f[0] + a * B
        self.phi.append(phi)
        if xr2_dot in phi.free_symbols:  # ! This needs to be revised
            return phi
        else:
            return self.get_expr(phi, f, g, states, states_dot)

    def decompose_G_h(self, expr, g, states_dot):
        G = []
        h = 0
        for arg in expr.args:
            if xr2_dot in arg.free_symbols:
                # Shakiba: This is hard coded needs to change for a different model
                G = - arg.subs(xr2_dot, 1)
            else:
                h = h + arg
        return G, h


def nimble_car_c(x, params):
    # Controller for nimble car
    goal_x = params['goal_x']
    bad_sets = params['bad_sets']
    ctrl_param = params['ctrl_param']
    myCBF = params['myCBF']

    # Reference controller
    theta_ref = math.atan((goal_x[1]-x[1])/(goal_x[0]-x[0]))
    uref_0 = ctrl_param[0] * (theta_ref - x[2])

    # math.atan2(sin(theta_ref-x[2]), cos(theta_ref-x[2]))

    ############################
    # cvxopt quadratic program
    # minimize  0.5 x'Px + q'x
    # s.t       Gx<=h
    ############################
    # P matrix
    P = cvxopt.matrix(np.eye(1))
    # P = .5 * (P + P.T)  # symmetric

    # q matrix
    q = cvxopt.matrix(np.array([-1*uref_0]), (1, 1))

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
        print("QP iteration fail. Trying again...")
    # print(x, ' G: ', G, ' h: ', h, ' x_sol: ', x_sol)
    return x_sol[0:1]


def nimble_car_f(t, x, u, params):
    # Function for a silly bug
    # if goal reached, do nothing
    goal_x = params['goal_x']
    if (x[0] - goal_x[0])**2 + (x[1] - goal_x[1])**2 <= 0.1**2:
        return [0, 0, 0]

    # compute control given current position
    u_0 = nimble_car_c(x, params)

    # compute change in xy direction
    dx0 = math.cos(x[2])
    dx1 = math.sin(x[2])
    dx2 = u_0[0]

    return [dx0, dx1, dx2]


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


# Robot Goal
goal_x = np.array([5, 5])

# Elipse format (x,y,rad_x,rad_y)
bad_sets = example(2)

# Parameters for reference controller
ctrl_param = [5]

xr0, xr1, xr2, cx, cy, rad_x, rad_y, xr2_dot, u = symbols(
    'xr0 xr1 xr2 cx cy rad_x rad_y xr2_dot u')

B = ((xr0 - cx)/rad_x)**2 + ((xr1 - cy)/rad_y)**2 - 1
# B = (xr0 - cx)**2 + (xr1 - cx)**2 - 1
f = Matrix([cos(xr2), sin(xr2), 0])
g = Matrix([0, 0, 1])
states_dot = Matrix([cos(xr2), sin(xr2), xr2_dot])

# expr_bs_dx0 = diff(expr_bs,xr0)
# expr_bs_dx1 = diff(expr_bs,xr1)

myCBF = CBF(B, f, g, (xr0, xr1, xr2), bad_sets, states_dot)

#? Simulation settings
T_max = 10
n_samples = 500
T = np.linspace(0, T_max, n_samples)
dt = T[1]-T[0]
params = {'goal_x': goal_x, 'bad_sets': bad_sets,
          'ctrl_param': ctrl_param, 'myCBF': myCBF}

# Disable cvxopt optimiztaion output
cvxopt.solvers.options['show_progress'] = False
# cvxopt.solvers.options['max_iter'] = 1000

# Plot
fig, ax = plt.subplots()

# Loop through initial conditions
print('Computing trajectories for initial conditions:')
print('x_0\t x_1')


curr_bs = []
for idxi, _ in enumerate(bad_sets):
    curr_bs = bad_sets[idxi]
    ell = Ellipse((curr_bs[0], curr_bs[1]), 2 *
                  curr_bs[2], 2 * curr_bs[3], color='r')
    ax.add_patch(ell)

# goal_square = plt.Rectangle(goal_x-np.array([.1, .1]), .2, .2, color='g')
# ax.add_patch(goal_square)

goal_circle = Ellipse((goal_x[0], goal_x[1]), 2*0.1, 2*0.1, color='g')
ax.add_patch(goal_circle)

plt.xlim(-2, 7)
plt.ylim(-2, 7)

# If initial condition is inside the bad set, skip it.
bool_val = 0
curr_bs = []

x_0 = np.array([0.5, 0.5, 0])

for idxj, j in enumerate(bad_sets):
    curr_bs = bad_sets[idxj]
    assert is_inside_ellipse(
        [x_0[0], x_0[1]], bad_sets[idxj]) == 0, "Initial condition is inside ellipse"


# Compute output on the silly bug system for given initial conditions and timesteps T
x = np.zeros((np.size(x_0), len(T)))
x[:, 0] = x_0
for i in range(len(T)-1):
    x[:, i+1] = x[:, i] + dt * \
        np.array(nimble_car_f(T[i], x[:, i], [], params))

line1, = ax.plot([], [], lw=2)
goal_square = plt.Rectangle(
    goal_x-np.array([.5, .5]), .2, .2, color='r', alpha=0.5)


def init():
    line1.set_data([], [])
    return line1


def animate(i):
    line1.set_data((x[0][0:i], x[1][0:i]))
    return line1, goal_square


ani = animation.FuncAnimation(
    fig, animate, init_func=init, interval=5, frames=n_samples, repeat=False)

plt.show()

# lgLfB(x) =  + ((-2*cy + 2*xr1)*cos(xr2)/rad_y**2 - (-2*cx + 2*xr0)*sin(xr2)/rad_x**2)*xr2_dot
# Lf^2B(x) =  + 2*cos(xr2)/rad_x**2)*cos(xr2) + 2*sin(xr2)/rad_y**2)*sin(xr2)

# \alpha_1 * \dot{b(x)} = 20*(-2*cx + 2*xr0)*cos(xr2)/rad_x**2 + 20*(-2*cy + 2*xr1)*sin(xr2)/rad_y**2 
# \alpha_1*\alpha_2B(x) = + 100*(-cx + xr0)**2/rad_x**2 + 100*(-cy + xr1)**2/rad_y**2 - 100



#  + 10*(-2*cx + 2*xr0)*cos(xr2)/rad_x**2 
#  + 10*(-2*cy + 2*xr1)*sin(xr2)/rad_y**2 
#  + (10*(-2*cx + 2*xr0)/rad_x**2)*cos(xr2) 
#  + (10*(-2*cy + 2*xr1)/rad_y**2)*sin(xr2)



# 2*(-50*rad_x**2*rad_y**2 + rad_x**2*(10*(-cy + xr1)*sin(xr2)

#2*(-50*rad_x**2*rad_y**2 + rad_x**2*(10*(-cy + xr1)*sin(xr2) + 50*(cy - xr1)**2 - (10*cy - 10*xr1 - sin(xr2))*sin(xr2)) + rad_y**2*(10*(-cx + xr0)*cos(xr2) + 50*(cx - xr0)**2 - (10*cx - 10*xr0 - cos(xr2))*cos(xr2)) - xr2_dot*(rad_x**2*(cy - xr1)*cos(xr2) - rad_y**2*(cx - xr0)*sin(xr2)))/(rad_x**2*rad_y**2)