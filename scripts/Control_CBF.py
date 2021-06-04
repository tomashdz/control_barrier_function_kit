# from HSR_control import get_model_srv
from gazebo_msgs.srv import GetWorldProperties, GetModelState, GetModelStateRequest
import numpy as np
import rospy

     

class Control_CBF(object):               
        def __init__(self, connected_system, goal_func, MapInfo , P = None, Q = None, IncludeRadius = 10):  # make sure ego is the system with which CBF is created 
                self.ego = connected_system.ego
                self.CBFList = connected_system.CBFList
                self.GoalInfo = goal_func
                self.MapInfo = MapInfo
                self.P = 1
                self.Q = 1
                self.IncludeRadius = IncludeRadius
                self.count = 0 # num of times control_callback is called
                self.flag = 0


        def controller_callback(self, event):
                # this controller loop call back.
                self.count += 1


                #         # making vw data and publish it.
                #         vel_msg = Twist()
                #         # Compute controller
                #         if abs(p.x)<1.5 and self.flag == 0:
                #                 self.flag = 1
                #                 env_bounds = type('', (), {})()
                #                 env_bounds.x_max = 1.2 
                #                 env_bounds.x_min = -1.3
                #                 self.MapInfo = self.robot.MapFuncs(env_bounds)
                #                 GoalCenter = np.array([0, 5.5])
                #                 self.GoalInfo = self.robot.GoalFuncs(GoalCenter,rGoal)

                u = self.cbf_controller_compute()
                self.ego.publish(u)                         

                #         if self.count > 1000:
                #                 rospy.loginfo('reach counter!!')
                #                 rospy.signal_shutdown('reach counter')
                #         elif self.GoalInfo.set(x_r)<0:
                #                 rospy.loginfo('reached GoalInfo set!!')
                #                 rospy.signal_shutdown('reached GoalInfo set')

        def cbf_controller_compute(self):
                x_r = self.ego.currState
                x_o = []
                for CBF in self.CBFList:
                        x_o.append(CBF.agent.currState)
                u_s = self.ego.inputs
                # if self.count>3:
                # x_o_pre = np.array(self.trajs.actors[len(self.trajs.actors)-4])
                # # x_o_2pre = np.array(self.trajs.actors[len(self.trajs.actors)-3])
                # dt = self.trajs.time[len(self.trajs.time)-1]-self.trajs.time[len(self.trajs.time)-4]
                # u_o = (x_o[:,0:2]-x_o_pre[:,0:2])/dt
                # else:
                # u_o = np.zeros((len(x_o),len(self.robot.u_o)))

                CBFList = self.CBFList
                GoalInfo = self.GoalInfo
                Map = self.MapInfo
                
                UnsafeList = []
                Dists = []
                for CBF in CBFList:
                        Dist = CBF.h(x_r, CBF.agent.currState)
                        Dists.append(Dist)
                        if Dist<self.IncludeRadius:
                                UnsafeList.append(CBF)
                        ai = 1
                if min(Dists)<0:
                        InUnsafe = 1
                else:
                        InUnsafe = 0
                minDist = min(Dists)
                minJ = np.where(Dists == minDist)

                numQPvars = len(u_s)+2*len(UnsafeList)+len(Map.h)+1
                numConstraints = 2*len(u_s)+2*len(UnsafeList)+len(Map.h)+2

                A = np.zeros((numConstraints,numQPvars))
                b = np.zeros(numConstraints)
                
                for j in range(len(UnsafeList)):
                        # CBF Constraints        
                        A[2*j, np.arange(len(u_s))]  = UnsafeList[j].LHS(x_r, UnsafeList[j].agent.currState)[0]
                        A[2*j, len(u_s)+2*j] = -1
                        b[2*j] = UnsafeList[j].RHS(x_r, UnsafeList[j].agent.currState)  
                        A[2*j+1, len(u_s)+2*j] = -1
                        b[2*j+1] = 0



                
                # Adding U constraint
                A[2*len(UnsafeList),0] = 1; b[2*len(UnsafeList)] = self.ego.inputRange[0,1]
                A[2*len(UnsafeList)+1,0] = -1;  b[2*len(UnsafeList)+1] = -self.ego.inputRange[0,0]
                A[2*len(UnsafeList)+2,1] = 1; b[2*len(UnsafeList)+2] = self.ego.inputRange[1,1]
                A[2*len(UnsafeList)+3,1] = -1; b[2*len(UnsafeList)+3] = -self.ego.inputRange[1,0]
                
                # Adding map constraints
                for j in range(len(Map.set)):
                        A[2*len(UnsafeList)+2*len(u_s)+j,np.append(np.arange(len(u_s)),[len(u_s)+2*len(UnsafeList)+j])] = [Map.setDer[j](x_r,[1, 0]), Map.setDer[j](x_r,[0, 1]), -1]
                        b[2*len(UnsafeList)+2*len(u_s)+j] = -Map.CBF[j](x_r)

                # Adding GoalInfo based Lyapunov !!!!!!!!!!!!!!!!! Needs to be changed for a different example 
                A[2*len(UnsafeList)+2*len(u_s)+len(Map.set),0:2] = [GoalInfo.Lyap(x_r,[1,0]), GoalInfo.Lyap(x_r,[0, 1])]
                A[2*len(UnsafeList)+2*len(u_s)+len(Map.set),-1] = -1
                b[2*len(UnsafeList)+2*len(u_s)+len(Map.set)] = 0
                A[2*len(UnsafeList)+2*len(u_s)+len(Map.set)+1,-1] = 1
                b[2*len(UnsafeList)+2*len(u_s)+len(Map.set)+1] = np.finfo(float).eps+1
                
                H = np.zeros((len(u_s)+2*len(UnsafeList)+len(Map.set)+1,len(u_s)+2*len(UnsafeList)+len(Map.set)+1))
                H[0,0] = 0
                H[1,1] = 0

                ff = np.zeros((len(u_s)+2*len(UnsafeList)+len(Map.set)+1,1))
                for j in range(len(UnsafeList)):
                        ff[len(u_s)+2*j] = 65
                        H[len(u_s)+2*j+1,len(u_s)+2*j+1] = 10000
                        # ff[len(u_s)+2*j+1] = 50* CBFList.CBF(x_r, x_o[minJ[0][0]][0:2])

                ff[len(u_s)+2*len(UnsafeList):len(u_s)+2*len(UnsafeList)+len(Map.set)] = 20
                ff[-1] = np.ceil(self.count/100.0)
        

                try:
                        uq = cvxopt_solve_qp(H, ff, A, b)
                except ValueError:
                        uq = [0,0]
                        rospy.loginfo('Domain Error in cvx')

                if uq is None:
                        uq = [0,0]
                        rospy.loginfo('infeasible QP')
                
                if findBestCommandAnyway and len(uq[2:len(uq)-2*len(Map.set)-1:2])>0:   # If humans are around and findbestcommand active
                        if InUnsafe:
                                self.trajs.risk.append(1.0)
                        else:
                                r = np.zeros(len(uq[2:len(uq)-2*len(Map.set)-1:2]))
                                for k in range(len(uq[2:len(uq)-2*len(Map.set)-1:2])):
                                        r[k] = min(1, max(0,1-(1-CBFList.CBF(x_r, x_o[UnsafeList[k]][0:2]))*exp(-uq[2*k+2]*T)))
                                self.trajs.risk.append(max(r))    
                elif not findBestCommandAnyway and len(uq[2:len(uq)-len(Map.set)-1])>0:
                        r = np.zeros(len(uq[2:len(uq)-len(Map.set)-1]))
                        for k in range(len(uq[2:len(uq)-len(Map.set)-1])):
                                r[k] = min(1, max(0,1-(1-CBFList.CBF(x_r, x_o[UnsafeList[k]][0:2]))*exp(-uq[k+2]*T)))
                        self.trajs.risk.append(max(r))
                        if max(r)>0.1:
                                1
                elif not findBestCommandAnyway and len(uq) == 2:  # feasible solution is not found
                        self.trajs.risk.append(-risk)  # meaning that solution is not found 
                else:  # No human is around 
                        self.trajs.risk.append(0.0)    
                self.trajs.minDist.append(minDist)

                return uq
