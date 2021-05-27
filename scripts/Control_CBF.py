from runScript import get_model_srv
from gazebo_msgs.srv import GetWorldProperties, GetModelState, GetModelStateRequest
import numpy as np

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



def get_agent_pose(actors)
    actors_data = []
    for actor in actors:
            model_actor = GetModelStateRequest()
            model_actor.model_name = actor
            model_actor = get_model_srv(model_actor) # the pose date is based on /map
            # actor_base_footprint_pose = self.gazebo_pos_transformPose('base_footprint', model_actor) # trasfer /map->/base_footprint
            angular = orientation2angular(model_actor.pose.orientation)      # transfer orientaton(quaternion)->agular(euler)
            p = model_actor.pose.position
            actors_data.append([p.x,p.y, angular.z])
    return actors_data       

class Control_CBF(object):
    def __init__(self, ego, CBFList, goal_set_func, MapInfo , P = None, Q = None, IncludeRadius = 10):  # make sure ego is the system with which CBF is created 
        self.ego = ego
        self.CBFList = CBFList
        self.GoalInfo = self.GoalFuncs(goal_set_func,ego)
        self.MapInfo = self.MapInfo
        self.P = 1
        self.Q = 1
        self.IncludeRadius = IncludeRadius
        self.count = 0 # num of times control_callback is called
        self.flag = 0


    def GoalFuncs(self,goal_set_func,ego):
        GoalInfo = type('', (), {})()
        GoalInfo.set = goal_set_func
        GoalSym = goal_set_func(*ego.states)
        GoalInfo.Lyap = lambdify([ego.states,ego.inputs],GoalSym.diff(ego.states).T*ego.dx)
        return GoalInfo

    def update_odometry(self,odometry):
        self.odometry = odometry
    def update_poseStamped(self,poseStamped):
        self.poseStamped = poseStamped
    def update_actors_data(self, actors):
        self.actors_data = get_agent_pose(actors)
    def set_goal(self, goal_set_func):
        self.goal_set_func = goal_set_func

   
    def update_states(self):
        now = rospy.get_rostime()
        time = now.secs+now.nsecs*pow(10,-9)
        if DEBUG:
                rospy.loginfo('Current time %i %i', now.secs, now.nsecs)
                rospy.loginfo('tOdometry\n %s', self.connected_ego.odometry) 
        self.connected_ego.system.add_state_traj(state, time)

    def controller_callback(self, event):
                # this controller loop call back.
                self.count += 1
                # [time, update_states()


        #         now = rospy.get_rostime()
        #         self.trajs.time.append(now.secs+now.nsecs*pow(10,-9))
        #         if DEBUG:
        #                 rospy.loginfo('Current time %i %i', now.secs, now.nsecs)
        #                 rospy.loginfo('tOdometry\n %s', self.odometry)
        #         # get human model state from Gazebo
        #         actors_data = []
        #         for actor in self.actors:
        #                 model_actor = GetModelStateRequest()
        #                 model_actor.model_name = actor
        #                 model_actor = self.get_model_srv(model_actor) # the pose date is based on /map
        #                 # actor_base_footprint_pose = self.gazebo_pos_transformPose('base_footprint', model_actor) # trasfer /map->/base_footprint
        #                 angular = orientation2angular(model_actor.pose.orientation)      # transfer orientaton(quaternion)->agular(euler)
        #                 p = model_actor.pose.position
        #                 actors_data.append([p.x,p.y, angular.z])
        #                 if DEBUG:
        #                         rospy.loginfo('%s in timestamp:\n%s', actor, model_actor.header.stamp) # time stamp is here.
        #                         rospy.loginfo('%s in base_footprint\nposition:\n%s\nangular:\n%s', actor, actor_base_footprint_pose.pose.position, angular)
        #         self.trajs.actors.append(actors_data)


        #         # get hsr model state from odometry
        #         model_hsr = self.odometry
        #         p = model_hsr.pose.pose.position
        #         angular = orientation2angular(model_hsr.pose.pose.orientation)      # transfer orientaton(quaternion)->agular(euler)
        #         x_r = [p.x,p.y,angular.z]
        #         self.trajs.hsr.append(x_r)

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



            # u = self.cbf_controller_compute()
        #         vel_msg.linear.x  = u[0]
        #         vel_msg.angular.z = u[1]
        #         self.vw_publisher.publish(vel_msg)
        #         self.trajs.commands.append([u[0],u[1]])
                               

        #         if self.count > 1000:
        #                 rospy.loginfo('reach counter!!')
        #                 rospy.signal_shutdown('reach counter')
        #         elif self.GoalInfo.set(x_r)<0:
        #                 rospy.loginfo('reached GoalInfo set!!')
        #                 rospy.signal_shutdown('reached GoalInfo set')


    # def tOdometry_callback(self, odometry):
    #     self.odometry = odometry # this odometry's coodination is \map

    # def odometry_callback(self, poseStamped):
    #     self.poseStamped = poseStamped
    
    # def gazebo_pos_transformPose(self, frame_id, gazebo_pose):
    #     gazebo_pose_temp = PoseStamped()
    #     gazebo_pose_temp.header = gazebo_pose.header
    #     gazebo_pose_temp.header.frame_id = 'map'
    #     gazebo_pose_temp.pose = gazebo_pose.pose
    #     while not rospy.is_shutdown():
    #             try:
    #                     gazebo_pos_trans = self.tfListener.transformPose(frame_id, gazebo_pose_temp)
    #                     break
    #             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #                     continue

    #     return gazebo_pos_trans


 def cbf_controller_compute(self):
        pass
        x_r = np.array([1, 1, 1])
        x_o = np.array([[ 4.75      , -1.01058068,  1.54978755],
       [-5.98917863,  1.00841121, -1.58917276],
       [-4.0251211 ,  1.01337521, -1.59609867],
       [ 1.19996358,  3.33624032, -1.73547199],
       [ 0.42953054,  3.7313696 ,  0.836555  ],
       [ 0.68975245,  2.95885104,  2.776555  ]])
        u_s = self.ego.inputs

        # if self.count>3:
        #     x_o_pre = np.array(self.trajs.actors[len(self.trajs.actors)-4])
        #     # x_o_2pre = np.array(self.trajs.actors[len(self.trajs.actors)-3])
        #     dt = self.trajs.time[len(self.trajs.time)-1]-self.trajs.time[len(self.trajs.time)-4]
        #     u_o = (x_o[:,0:2]-x_o_pre[:,0:2])/dt
        # else:
        #     u_o = np.zeros((len(x_o),len(self.robot.u_o)))
        ego = self.ego
        CBFList = self.CBFList
        GoalInfo = self.GoalInfo
        # Map = self.MapInfo
        UnsafeList = []
        Dists = np.zeros((len(CBFList)))
        for CBF in CBFList:
            Dists[j] = CBF.h(ego.currState, CBF.agent.currState)
            if Dists[j]<self.IncludeRadius:
                    UnsafeList.append(CBF)
        ai = 1
        if min(Dists)<0:
                InUnsafe = 1
        else:
                InUnsafe = 0
        minDist = min(Dists)
        minJ = np.where(Dists == minDist)

        numQPvars = len(u_s)+2*len(UnsafeList)+len(Map.set)+1
        numConstraints = 2*len(u_s)+2*len(UnsafeList)+len(Map.set)+2
        A = np.zeros((numConstraints,numQPvars))
        b =np.zeros(numConstraints)
        for j in range(len(UnsafeList)):
            # CBF Constraints        
            A[2*j,np.append(np.arange(len(u_s)),[len(u_s)+2*j])] = [CBFList.multCond(x_r,  x_o[UnsafeList[j]][0:2],[1, 0]), CBFList.multCond(x_r,x_o[UnsafeList[j]][0:2],[0, 1]), -1] # multiplier of u , bi
            b[2*j] = -ai* CBFList.CBF(x_r, x_o[UnsafeList[j]][0:2])- CBFList.ConstCond(x_r,  x_o[UnsafeList[j]][0:2],u_o[UnsafeList[j]]) 
            # Constraints on bi to satisfy pi risk
            A[2*j+1,len(u_s)+2*j] = 1; A[2*j+1,len(u_s)+2*j+1] = -1 
            if CBFList.CBF(x_r, x_o[UnsafeList[j]][0:2])<1:
                    b[2*j+1] = min(ai, -1/T*log((1-risk)/(1-CBFList.CBF(x_r, x_o[UnsafeList[j]][0:2]))))
            else:
                    b[2*j+1] = 0



        
        # Adding U constraint
        A[2*len(UnsafeList),0] = 1; b[2*len(UnsafeList)] = U[0,1]
        A[2*len(UnsafeList)+1,0] = -1;  b[2*len(UnsafeList)+1] = -U[0,0]
        A[2*len(UnsafeList)+2,1] = 1; b[2*len(UnsafeList)+2] = U[1,1]
        A[2*len(UnsafeList)+3,1] = -1; b[2*len(UnsafeList)+3] = -U[1,0]
        
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
