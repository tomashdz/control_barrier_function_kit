#! /usr/bin/env python
# call roscore
# $ roscore
#
# If start in manual
# $ rosrun cbf_ros cbf_controller.py
import rospy
import sys
import argparse
import re
import numpy as np
from scipy.integrate import odeint
from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log
import matplotlib.pyplot as plt  
import cvxopt as cvxopt

# ROS msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetWorldProperties, GetModelState, GetModelStateRequest

# ROS others
import tf

DEBUG = False

def orientation2angular(orientation):
        quaternion = (  orientation.x,
                        orientation.y,
                        orientation.z,
                        orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        angular = Vector3(
                euler[0],
                euler[1],
                euler[2]
        )
        return angular

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

class robot(object):
        def __init__(self,l):
               #Symbolic Variables
                # t = symbols('t')
                # when robot is bicycle model [x,y,theta], obstacles are linear models [x,y]:
                xr1,xr2,xr3,xo1,xo2 = symbols('xr1 xr2 xr3 xo1 xo2')  
                # v w inputs of robot:
                u1,u2 = symbols('u1,u2')
                # Vector of states and inputs:
                self.x_r_s = Matrix([xr1,xr2,xr3])
                self.x_o_s = Matrix([xo1,xo2])
                self.u_s = Matrix([u1,u2]) 

                self.f = Matrix([0,0,0])
                self.g = Matrix([[cos(self.x_r_s[2]), -l*sin(self.x_r_s[2])], [sin(self.x_r_s[2]), l*cos(self.x_r_s[2])], [0, 1]])
                self.f_r = self.f+self.g*self.u_s 
                self.l = l #approximation parameter for bicycle model
                self.Real_x_r = lambdify([self.x_r_s], self.x_r_s-Matrix([l*cos(self.x_r_s[2]), l*sin(self.x_r_s[2]), 0]))
                
                # Obstacle SDE, not needed if we want to use Keyvan prediction method
                self.f_o = Matrix([1.5,0])
                self.g_o = Matrix([0.2, 0])

                self.f_o_fun = lambdify([self.x_o_s], self.f_o)
                self.g_o_fun = lambdify([self.x_o_s], self.g_o)
        
        def GoalFuncs(self,GoalCenter,rGoal):
                Gset = (self.x_r_s[0]-GoalCenter[0])**2+(self.x_r_s[1]-GoalCenter[1])**2-rGoal
                GoalInfo = type('', (), {})()
                GoalInfo.set = lambdify([self.x_r_s],Gset)
                GoalInfo.Lyap = lambdify([self.x_r_s,self.u_s],Gset.diff(self.x_r_s).T*self.f_r)
                return GoalInfo

        def UnsafeFuncs(self,gamma,UnsafeRadius):   #based on the SDE formulation, needs slight change for regular BF
                UnsafeInfo = type('', (), {})()
                Uset = (self.x_r_s[0]-self.x_o_s[0])**2+(self.x_r_s[1]-self.x_o_s[1])**2-(UnsafeRadius+self.l)**2
                CBF = exp(-gamma*Uset)
                CBF_d = CBF.diff(Matrix([self.x_r_s,self.x_o_s]))
                CBF_d2 =  CBF.diff(self.x_o_s,2)
                UnsafeInfo.set = lambdify([self.x_r_s,self.x_o_s], Uset)
                UnsafeInfo.CBF = lambdify([self.x_r_s,self.x_o_s], CBF)
                UnsafeInfo.ConstCond = lambdify([self.x_r_s,self.x_o_s] , CBF_d.T*Matrix([self.f,self.f_o])+0.5*(self.g_o.T*Matrix([[Matrix(CBF_d2[0,0]),Matrix(CBF_d2[1,0])]])*self.g_o))
                UnsafeInfo.multCond = lambdify([self.x_r_s,self.x_o_s,self.u_s], CBF_d.T*Matrix([self.g*self.u_s, Matrix(np.zeros((len(self.x_o_s),1)))]))
                return UnsafeInfo

        def MapFuncs(self,env_bounds):
                MapInfo = type('', (), {})()
                MapInfo.set = []
                MapInfo.setDer = []
                # x_min = getattr(env_bounds, "x_min", undefined)
                # x_max = getattr(env_bounds, "x_max", undefined)
                # y_min = getattr(env_bounds, "y_min", undefined)
                # y_max = getattr(env_bounds, "y_max", undefined)
                if hasattr(env_bounds,'x_min'):
                        Uset = (self.x_r_s[0]-env_bounds.x_min)
                        MapInfo.set.append(lambdify([self.x_r_s], Uset))
                        MapInfo.setDer.append(lambdify([self.x_r_s,self.u_s] , Uset.diff(self.x_r_s).T*self.f_r))
                if hasattr(env_bounds,'x_max'):
                        Uset = (-self.x_r_s[0]+env_bounds.x_max)
                        MapInfo.set.append(lambdify([self.x_r_s], Uset))
                        MapInfo.setDer.append(lambdify([self.x_r_s,self.u_s] , Uset.diff(self.x_r_s).T*self.f_r))
                if hasattr(env_bounds,'y_min'):
                        Uset = (self.x_r_s[1]-env_bounds.y_min)
                        MapInfo.set.append(lambdify([self.x_r_s], Uset))
                        MapInfo.setDer.append(lambdify([self.x_r_s,self.u_s] , Uset.diff(self.x_r_s).T*self.f_r))                                
                if hasattr(env_bounds,'y_max'):
                        Uset = (-self.x_r_s[1]+env_bounds.y_max)
                        MapInfo.set.append(lambdify([self.x_r_s], Uset))
                        MapInfo.setDer.append(lambdify([self.x_r_s,self.u_s] , Uset.diff(self.x_r_s).T*self.f_r))                        
                return MapInfo
                



                
                


class CBF_CONTROLLER(object):
	def __init__(self,robot,GoalInfo,UnsafeInfo):
                # publisher to send vw order to HSR
                self.vw_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
                # subscriber for Gazebo info.
                rospy.wait_for_service ('/gazebo/get_model_state')
                self.get_model_pro = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
                self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                self.tOdometry_subscriber = rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, self.tOdometry_callback, queue_size=10)
                self.tOdometry = Odometry()
                self.odometry_subscriber = rospy.Subscriber('/global_pose', PoseStamped, self.odometry_callback, queue_size=10)
                self.poseStamped = PoseStamped()
                # listener of tf.
                self.tfListener = tf.TransformListener()


                trajs = type('', (), {})()
                trajs.hsr = []
                trajs.actors = []
                trajs.commands = []
                trajs.time = []
                self.trajs = trajs
                self.robot = robot
                self.GoalInfo = GoalInfo
                self.UnsafeInfo = UnsafeInfo
                self.MapInfo = MapInfo
                self.flag = 0
                self.count = 0 # num of times control_callback is called

        def __del__(self):
                pass

        def tOdometry_callback(self, odometry):
                self.odometry = odometry # this odometry's coodination is \map

        def odometry_callback(self, poseStamped):
                self.poseStamped = poseStamped
        
        def gazebo_pos_transformPose(self, frame_id, gazebo_pose):
                gazebo_pose_temp = PoseStamped()
                gazebo_pose_temp.header = gazebo_pose.header
                gazebo_pose_temp.header.frame_id = 'map'
                gazebo_pose_temp.pose = gazebo_pose.pose
                while not rospy.is_shutdown():
                        try:
                                gazebo_pos_trans = self.tfListener.transformPose(frame_id, gazebo_pose_temp)
                                break
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                continue

                return gazebo_pos_trans

        def controller_loop_callback(self, event):
                # this controller loop call back.
                self.count += 1
                now = rospy.get_rostime()
                # self.trajs.time.append(now)
                if DEBUG:
                        rospy.loginfo('Current time %i %i', now.secs, now.nsecs)
                        rospy.loginfo('tOdometry\n %s', self.odometry)
                # get human model state from Gazebo
                model_properties = self.get_model_pro()
                actors = []
                for model_name in model_properties.model_names:
                        if re.search('actor*', model_name):  # if the model name is actor*, it will catch them.
                                actors.append(model_name) 
                actors_data = []
                for actor in actors:
                        model_actor = GetModelStateRequest()
                        model_actor.model_name = actor
                        model_actor = self.get_model_srv(model_actor) # the pose date is based on /map
                        actor_base_footprint_pose = self.gazebo_pos_transformPose('base_footprint', model_actor) # trasfer /map->/base_footprint
                        angular = orientation2angular(actor_base_footprint_pose.pose.orientation)      # transfer orientaton(quaternion)->agular(euler)
                        p = actor_base_footprint_pose.pose.position
                        actors_data.append([p.x,p.y, angular.z])
                        if DEBUG:
                                rospy.loginfo('%s in timestamp:\n%s', actor, model_actor.header.stamp) # time stamp is here.
                                rospy.loginfo('%s in base_footprint\nposition:\n%s\nangular:\n%s', actor, actor_base_footprint_pose.pose.position, angular)
                self.trajs.actors.append(actors_data)


                # get hsr model state from odometry
                model_hsr = self.odometry
                p = model_hsr.pose.pose.position
                angular = orientation2angular(model_hsr.pose.pose.orientation)      # transfer orientaton(quaternion)->agular(euler)
                x_r = [p.x,p.y,angular.z]
                self.trajs.hsr.append(x_r)

                # making vw data and publish it.
                vel_msg = Twist()
                # Compute controller
                u = self.cbf_controller_compute()
                vel_msg.linear.x  = u[0]
                vel_msg.angular.z = u[1]
                self.vw_publisher.publish(vel_msg)
                self.trajs.commands.append(u)
                               

                if self.count > 500:
                        rospy.loginfo('reach counter!!')
                        rospy.signal_shutdown('reach counter')
                
                
        def cbf_controller_compute(self):
                x_r = np.array(self.trajs.hsr[len(self.trajs.hsr)-1])
                x_o = np.array(self.trajs.actors[len(self.trajs.actors)-1])
                u_s = self.robot.u_s
                Unsafe = self.UnsafeInfo
                Goal = self.GoalInfo
                UnsafeList = []
                Dists = np.zeros((len(x_o)))
                for j  in range(len(x_o)):
                        Dists[j] = Unsafe.set(x_r,  x_o[j][0:2])
                        if Dists[j]<UnsafeInclude:
                                UnsafeList.append(j)
                ai = 1
                A = np.zeros((2*len(UnsafeList)+2*len(u_s)+2,len(u_s)+len(UnsafeList)+1))
                b =np.zeros((2*len(u_s)+2*len(UnsafeList)+2))
                for j in range(len(UnsafeList)):
                        # CBF Constraints        
                        A[2*j,np.append(np.arange(len(u_s)),[len(u_s)+j])] = [Unsafe.multCond(x_r,  x_o[UnsafeList[j]][0:2],[1, 0]), Unsafe.multCond(x_r,x_o[UnsafeList[j]][0:2],[0, 1]), -1] # multiplier of u , bi
                        b[2*j] = -ai* Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2])- Unsafe.ConstCond(x_r,  x_o[UnsafeList[j]][0:2])  
                        # Constraints on bi to satisfy pi risk
                        A[2*j+1,len(u_s)+j] = 1
                        b[2*j+1] = min(ai, -1/T*log((1-risk))/(1-Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2])))
                
                # Adding U constraint
                A[2*len(UnsafeList),0] = 1; b[2*len(UnsafeList)] = U[0,1]
                A[2*len(UnsafeList)+1,0] = -1;  b[2*len(UnsafeList)+1] = -U[0,0]
                A[2*len(UnsafeList)+2,1] = 1; b[2*len(UnsafeList)+2] = U[1,1]
                A[2*len(UnsafeList)+3,1] = -1; b[2*len(UnsafeList)+3] = -U[1,0]
                
                # Adding Goal based Lyapunov !!!!!!!!!!!!!!!!! Needs to be changed for a different example 
                A[2*len(UnsafeList)+2*len(u_s),0:2] = [Goal.Lyap(x_r,[1,0]), Goal.Lyap(x_r,[0, 1])]
                A[2*len(UnsafeList)+2*len(u_s),-1] = -1
                b[2*len(UnsafeList)+2*len(u_s)] = 0
                A[2*len(UnsafeList)+2*len(u_s)+1,-1] = 1
                b[2*len(UnsafeList)+2*len(u_s)+1] = np.finfo(float).eps

                H = np.zeros((len(u_s)+len(UnsafeList)+1,len(u_s)+len(UnsafeList)+1))  # u1, u2 , b1 to  b4 for obstacles, delta (for lyapunov)
                H[0,0] = 10
                H[1,1] = 0.5

                ff = np.zeros((len(u_s)+len(UnsafeList)+1,1))
                ff[len(u_s):len(u_s)+len(UnsafeList)] = 100
                if self.count<-1:    # Needs correction
                        if uq[0]>u1d:
                                ui = max(u1d,uq[0]-0.1)
                        else:
                                ui = min(u1d,uq[0]+0.1)
                else:
                        ui = u1d

                ff[0] = -10*ui
                ff[1] = 0.5*0.1*x_r[2]
                ff[-1] = 1
                uq = cvxopt_solve_qp(H, ff, A, b)
                if uq is None:
                        uq = [0,0]
                        rospy.loginfo('infeasible QP')

                return uq

if __name__ == '__main__':
        ## Parameters  
        # Goal info
        GoalCenter = np.array([0, 5])
        rGoal = np.power(0.1,2)
        # Unsafe 
        UnsafeInclude = 5    # consider obstacle if in radius
        UnsafeRadius = .5    #radius of unsafe sets/distance from obstacles
        # Enviroment Bounds
        env_bounds = type('', (), {})()
        env_bounds.y_min = -1.7
        env_bounds.y_max = 1.5 
        env_bounds.x_max = 1.55 
        env_bounds.x_min = -1.6

        l = 0.01   #bicycle model approximation parameter
        U = np.array([[-0.33,0.33],[-0.3,0.3]])
        T = 1  #Lookahead horizon
        risk = 0.1    #max risk desired        
        gamma = 5       #CBF coefficient
        u1d = 0  #desired input to save energy!
        # Plotting options 
        plotit = 1
        plotlanes = 1

        robot = robot(l)
        GoalInfo = robot.GoalFuncs(GoalCenter,rGoal)
        UnsafeInfo = robot.UnsafeFuncs(gamma,UnsafeRadius)
        MapInfo = robot.MapFuncs(env_bounds)   
   

        # Process arguments
        p = argparse.ArgumentParser(description='CBF controller')
        args = p.parse_args(rospy.myargv()[1:])

        
        try:
	        rospy.init_node('cbf_controller')
                cbf_controller = CBF_CONTROLLER(robot,GoalInfo,UnsafeInfo)
                control_priod = 0.1 #[sec] we can change controll priod with this parameter.
                rospy.Timer(rospy.Duration(control_priod), cbf_controller.controller_loop_callback)
                rospy.spin()

        except rospy.ROSInterruptException:
                pass

        print('You can put data save here')
