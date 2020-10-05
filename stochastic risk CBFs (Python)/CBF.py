import numpy as np
#import sympy as sym
from scipy.integrate import odeint
from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log
#from sympy.physics.quantum import TensorProduct
#from sympy.parsing.sympy_parser import (parse_expr, standard_transformations, implicit_multiplication)
#from sympy.abc import x,y,z,a,b
#from math import sin,cos
#from autograd import grad
import matplotlib.pyplot as plt  
import cvxopt as cvxopt
#from cvxopt import matrix, solvers


def cvxopt_solve_qp(P, q, G=None, h=None, A=None, b=None):
    P = .5 * (P + P.T)  # make sure P is symmetric
    args = [cvxopt.matrix(P), cvxopt.matrix(q)]
    if G is not None:
        args.extend([cvxopt.matrix(G), cvxopt.matrix(h)])
        if A is not None:
            args.extend([cvxopt.matrix(A), cvxopt.matrix(b)])
    sol = cvxopt.solvers.qp(*args)
    if 'optimal' not in sol['status']:
        return None
    return np.array(sol['x']).reshape((P.shape[1],))

#Symbolic Variables
t = symbols('t')
xr1,xr2,xr3,xo1,xo2 = symbols('xr1 xr2 xr3 xo1 xo2')
u1,u2 = symbols('u1,u2')
x_r_s = Matrix([xr1,xr2,xr3])
x_o_s = Matrix([xo1,xo2])
u_s = Matrix([u1,u2])

#Scenario parameters
x_r = np.array([0,0,0])
x_o = np.array([[-7,1],[-5,1],[-1, 1], [2.5,1],[5.5,1] ,[-10,2],  [-7,2],[-1,2], [0.45,2],[5,2], [-2,0], [3,0], [-.5, 3], [3,3],[-4,3]])
U = np.array([[0,2],[-np.pi/6,np.pi/6]])
T = 1
SimTime = 25
l = 0.01
u1d = 1.2
p = 0.1
GoalCenter = np.array([10, 3])
r = 0.5
rG = np.power(0.1,2)
rLane = np.power(0.5,2)
gamma = 5

#Dynamic & Stochastic systems
f = Matrix([0,0,0])
g = Matrix([[cos(x_r_s[2]), -l*sin(x_r_s[2])], [sin(x_r_s[2]), l*cos(x_r_s[2])], [0, 1]])
#f_r = f+np.matmul(g, u_s)
f_r = f+g*u_s

# f_fun = lambdify(x_r_s , f)
# g_fun = lambdify(x_r_s, g)
# f_r_fun = lambdify([t,x_r_s,u_s], f_r)
Real_x_r = lambdify([x_r_s], x_r_s-Matrix([l*cos(x_r_s[2]), l*sin(x_r_s[2]), 0]))

f_o = Matrix([1.5,0])
g_o = Matrix([0.2, 0])

f_o_fun = lambdify([x_o_s], f_o)
g_o_fun = lambdify([x_o_s], g_o)

# g = lambda x: np.matrix([[np.cos(x[2]), -l*np.sin(x[2])], [np.sin(x[2]), l*np.cos(x[2])], [0, 1]]);
# f_r = lambda t,x,u: f(x)+np.matmul(g(x), u);

# Plotting options 
plotit = 1
plotlanes = 1

# Unsafe and Goal sets and functions
class Unsafe:
    def init(self):
        self.Uset = []
        self.CBF = []
        self.multCond = []
        self.ConstCond = []
class Goal:
    def init(self):
        self.Gset = []
        self.Lyap = []       
        
Unsafe = Unsafe()    
Unsafe.init()
Goal = Goal()
Goal.init()

for i in range(len(x_o)):
    Uset = (x_r_s[0]-x_o_s[0])**2+(x_r_s[1]-x_o_s[1])**2-(r+l)**2
    CBF = exp(-gamma*Uset)
    CBF_d = CBF.diff(Matrix([x_r_s,x_o_s]))
    CBF_d2 =  CBF.diff(x_o_s,2)    
    Unsafe.Uset.append(lambdify([x_r_s,x_o_s], Uset))
    Unsafe.CBF.append(lambdify([x_r_s,x_o_s], CBF))
    Unsafe.ConstCond.append(lambdify([x_r_s,x_o_s] , CBF_d.T*Matrix([f,f_o])+0.5*(g_o.T*Matrix([[Matrix(CBF_d2[0,0]),Matrix(CBF_d2[1,0])]])*g_o)))
    Unsafe.multCond.append(lambdify([x_r_s,x_o_s,u_s], CBF_d.T*Matrix([g*u_s, Matrix(np.zeros((len(x_o_s),1)))])))

Gset = (x_r_s[1]-GoalCenter[1])**2-rG
Goal.Gset.append(lambdify([x_r_s],Gset))
Goal.Lyap.append(lambdify([x_r_s,u_s],Gset.diff(x_r_s).T*f_r))

#Obstacles
np.random.seed(1)
dt = 0.01
N =int(SimTime/dt)
dW = sqrt(dt)*np.random.normal(0, 1, (len(x_o), N))

x_o_traj = np.zeros((N,len(x_o_s),len(x_o)))
for j in range(len(x_o)):
    x_o_traj[0,:,j] = x_o[j]
    for i in range(N-1):
        x_o_traj[i+1,:,j] = x_o_traj[i,:,j] + np.squeeze(f_o_fun(x_o_traj[i,:,j])*dt + g_o_fun(x_o_traj[i,:,j])*dW[j,i]) 
        
#fig = plt.figure()
#plt.hold()
#plt.plot(np.arange(N),x_o_traj[:,0,:],color ="green")
#plt.show()


#QPs:

i = -1
UnsafeRadius = 3
curr_xr = x_r
x_r_traj = x_r
t_traj = 0
uq = []
fid = 0
#UnsafeLists = [];
bmax = np.zeros(N-1)
minDist = np.zeros(N-1)
risk= np.zeros(N-1)
u_r = np.zeros( (N-1 ,len(u_s)))
r_x_r = np.zeros( (N-1 ,len(x_r_s)))
while (Goal.Gset[0](curr_xr)>0 and i<N-1):
    i += 1
    UnsafeList = []
    Dists = np.zeros((len(x_o)))
    #options =  optimset('Display','off','MaxIter', 2000);
    for j  in range(len(x_o)):
        Dists[j] = Unsafe.Uset[j](curr_xr,  x_o_traj[i,:,j])
        if Dists[j]<UnsafeRadius:
            UnsafeList.append(j)
    
    #UnsafeLists = unique([UnsafeLists UnsafeList]);
    ai = 1
    A = np.zeros((2*len(UnsafeList)+2*len(u_s)+2,len(u_s)+len(UnsafeList)+1))
    b =np.zeros((2*len(u_s)+2*len(UnsafeList)+2))


    for j in range(len(UnsafeList)):
        # CBF Constraints        
        A[2*j,np.append(np.arange(len(u_s)),[len(u_s)+j])] = [Unsafe.multCond[UnsafeList[j]](curr_xr,  x_o_traj[i,:,UnsafeList[j]],[1, 0]), Unsafe.multCond[UnsafeList[j]](curr_xr,          x_o_traj[i,:,UnsafeList[j]],[0, 1]), -1] # multiplier of u , bi
        b[2*j] = -ai* Unsafe.CBF[UnsafeList[j]](curr_xr, x_o_traj[i,:,UnsafeList[j]])- Unsafe.ConstCond[UnsafeList[j]](curr_xr,  x_o_traj[i,:,UnsafeList[j]])  
        # Constraints on bi to satisfy pi risk
        A[2*j+1,len(u_s)+j] = 1
        b[2*j+1] = min(ai, -1/T*log((1-p))/(1-Unsafe.CBF[UnsafeList[j]](curr_xr, x_o_traj[i,:,UnsafeList[j]])))
    
    # Adding U constraint
    A[2*len(UnsafeList),0] = 1; b[2*len(UnsafeList)] = U[0,1]
    A[2*len(UnsafeList)+1,0] = -1;  b[2*len(UnsafeList)+1] = -U[0,0]
    A[2*len(UnsafeList)+2,1] = 1; b[2*len(UnsafeList)+2] = U[1,1]
    A[2*len(UnsafeList)+3,1] = -1; b[2*len(UnsafeList)+3] = -U[1,0]
    
    # Adding Goal based Lyapunov !!!!!!!!!!!!!!!!! Needs to be changed for a different example 
    A[2*len(UnsafeList)+2*len(u_s),0:2] = [Goal.Lyap[0](curr_xr,[1,0]), Goal.Lyap[0](curr_xr,[0, 1])]
    A[2*len(UnsafeList)+2*len(u_s),-1] = -1
    b[2*len(UnsafeList)+2*len(u_s)] = 0
    A[2*len(UnsafeList)+2*len(u_s)+1,-1] = 1
    b[2*len(UnsafeList)+2*len(u_s)+1] = np.finfo(float).eps

    H = np.zeros((len(u_s)+len(UnsafeList)+1,len(u_s)+len(UnsafeList)+1))  # u1, u2 , b1 to  b4 for obstacles, delta (for lyapunov)
    H[0,0] = 10
    H[1,1] = 0.5

    ff = np.zeros((len(u_s)+len(UnsafeList)+1,1))
    ff[len(u_s):len(u_s)+len(UnsafeList)] = 100
    if i>0:
        if uq[0]>u1d:
            ui = max(u1d,uq[0]-0.1)
        else:
            ui = min(u1d,uq[0]+0.1)
    else:
        ui = u1d

    ff[0] = -10*ui
    ff[1] = 0.5*0.1*curr_xr[2]
    ff[-1] = 1
    uq = cvxopt_solve_qp(H, ff, A, b)
    if uq is None:
        print('No feasible solution found')
        break    
    elif len(uq)>2:
        bmax[i] = max(uq[2:len(uq)-1])
        j = np.where(uq == bmax[i])
        r = np.zeros(len(uq)-3)
        for k in range(len(uq)-3):
            r[k] = max(0,1-(1-Unsafe.CBF[UnsafeList[k]](curr_xr, x_o_traj[i,:,UnsafeList[k]])*exp(-uq[k+2]*T)))
        risk[i] = max(r)
    else:
        bmax[i] = 0
        risk[i] = 0

    minDist[i] = min(Dists)
    curr_u = uq[0:len(u_s)]
    u_r[i,:] = curr_u
    Cl_fr = lambdify([x_r_s,t], (f+g*Matrix(curr_u)).T)
    def Cl_fr1(x,t):
        return list( np.squeeze(Cl_fr(curr_xr,t)))
    T_traj = np.linspace(i*dt, (i+1)*dt,20)
    sol = odeint(Cl_fr1, list(curr_xr), T_traj)
    curr_xr = sol[len(sol)-1,:]
    r_x_r[i+1,:] = np.squeeze(Real_x_r(curr_xr))
    plt.clf()
    plt.axis([-10,35,-1,4])
    plt.plot(r_x_r[i,0],r_x_r[i,1],color ="green",marker = 's')
    for kk in range(len(x_o)):
        plt.plot(x_o_traj[i,0,kk],x_o_traj[i,1,kk],color ="red",marker = 's')
    plt.draw()
    plt.pause(.00000000001)
plt.plot(r_x_r[0:i,0],r_x_r[0:i,1],color ="green")
plt.show()

