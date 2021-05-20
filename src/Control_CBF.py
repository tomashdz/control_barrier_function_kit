class data(object):
    def __init__(self):
        self.ego 
        self.agent
        self.time
        self.command
    def add_data(self, ego_state, agents_states, time, command):
        self.ego.append(ego_state)
        self.agent.append(agents_states)
        self.time.append(time)
        self.command.append(command)

class Control_CBF(object):
    def __init__(self, ego, CBF, goal_set_func, P = None, Q = None ):

        # # publisher to send vw order to HSR
        # self.vw_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        # # subscriber for Gazebo info.
        # rospy.wait_for_service ('/gazebo/get_model_state')
        # self.get_model_pro = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        # self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # self.tOdometry_subscriber = rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, self.tOdometry_callback, queue_size=10)
        # self.tOdometry = Odometry()
        # self.odometry_subscriber = rospy.Subscriber('/global_pose', PoseStamped, self.odometry_callback, queue_size=10)
        # self.poseStamped = PoseStamped()
        # # listener of tf.
        # self.tfListener = tf.TransformListener()

        # OR we can create another object/change it somehow s.t we do not need to change inside the class
        #like a sensing object or something like that


        self.ego = ego
        self.CBF = CBF
        self.goal_set_func = goal_set_func
        self.P = 0
        self.Q = 0
        self.data = data()
        
    def cbf_controller_compute(self,data):
        pass

    #     x_r = np.array(self.trajs.hsr[len(self.trajs.hsr)-1])
    #     x_o = np.array(self.trajs.actors[len(self.trajs.actors)-1])
    #     u_s = self.robot.u_s
    #     if self.count>3:
    #             x_o_pre = np.array(self.trajs.actors[len(self.trajs.actors)-4])
    #             # x_o_2pre = np.array(self.trajs.actors[len(self.trajs.actors)-3])
    #             dt = self.trajs.time[len(self.trajs.time)-1]-self.trajs.time[len(self.trajs.time)-4]
    #             u_o = (x_o[:,0:2]-x_o_pre[:,0:2])/dt
    #     else:
    #             u_o = np.zeros((len(x_o),len(self.robot.u_o)))

    #     Unsafe = self.UnsafeInfo
    #     Goal = self.GoalInfo
    #     Map = self.MapInfo
    #     UnsafeList = []
    #     Dists = np.zeros((len(x_o)))
    #     for j  in range(len(x_o)):
    #             Dists[j] = Unsafe.set(x_r,  x_o[j][0:2])
    #             if Dists[j]<UnsafeInclude:
    #                     UnsafeList.append(j)
    #     ai = 1
    #     if min(Dists)<0:
    #             InUnsafe = 1
    #     else:
    #             InUnsafe = 0
    #     minDist = min(Dists)
    #     minJ = np.where(Dists == minDist)

        
    #     if findBestCommandAnyway:
    #         #Ax<=b, x = [v, w , b1,bh1 b2, bh2..., bn, b'1, b'2,b'm, delta ]  
    #         # where b is constant in Eq (14) of paper "Risk-bounded  Control  using  Stochastic  Barrier  Functions"
    #         #b' is the slack variable for map constraints 
    #         # delta is for lyapunov function
    #         A = np.zeros((2*len(UnsafeList)+2*len(u_s)+len(Map.set)+2,len(u_s)+2*len(UnsafeList)+len(Map.set)+1))
    #         b =np.zeros((2*len(u_s)+2*len(UnsafeList)+len(Map.set)+2))
    #         for j in range(len(UnsafeList)):
    #                 # CBF Constraints        
    #                 A[2*j,np.append(np.arange(len(u_s)),[len(u_s)+2*j])] = [Unsafe.multCond(x_r,  x_o[UnsafeList[j]][0:2],[1, 0]), Unsafe.multCond(x_r,x_o[UnsafeList[j]][0:2],[0, 1]), -1] # multiplier of u , bi
    #                 b[2*j] = -ai* Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2])- Unsafe.ConstCond(x_r,  x_o[UnsafeList[j]][0:2],u_o[UnsafeList[j]]) 
    #                 # Constraints on bi to satisfy pi risk
    #                 A[2*j+1,len(u_s)+2*j] = 1; A[2*j+1,len(u_s)+2*j+1] = -1 
    #                 if Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2])<1:
    #                         b[2*j+1] = min(ai, -1/T*log((1-risk)/(1-Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2]))))
    #                 else:
    #                         b[2*j+1] = 0



            
    #         # Adding U constraint
    #         A[2*len(UnsafeList),0] = 1; b[2*len(UnsafeList)] = U[0,1]
    #         A[2*len(UnsafeList)+1,0] = -1;  b[2*len(UnsafeList)+1] = -U[0,0]
    #         A[2*len(UnsafeList)+2,1] = 1; b[2*len(UnsafeList)+2] = U[1,1]
    #         A[2*len(UnsafeList)+3,1] = -1; b[2*len(UnsafeList)+3] = -U[1,0]
            
    #         # Adding map constraints
    #         for j in range(len(Map.set)):
    #                 A[2*len(UnsafeList)+2*len(u_s)+j,np.append(np.arange(len(u_s)),[len(u_s)+2*len(UnsafeList)+j])] = [Map.setDer[j](x_r,[1, 0]), Map.setDer[j](x_r,[0, 1]), -1]
    #                 b[2*len(UnsafeList)+2*len(u_s)+j] = -Map.CBF[j](x_r)

    #         # Adding Goal based Lyapunov !!!!!!!!!!!!!!!!! Needs to be changed for a different example 
    #         A[2*len(UnsafeList)+2*len(u_s)+len(Map.set),0:2] = [Goal.Lyap(x_r,[1,0]), Goal.Lyap(x_r,[0, 1])]
    #         A[2*len(UnsafeList)+2*len(u_s)+len(Map.set),-1] = -1
    #         b[2*len(UnsafeList)+2*len(u_s)+len(Map.set)] = 0
    #         A[2*len(UnsafeList)+2*len(u_s)+len(Map.set)+1,-1] = 1
    #         b[2*len(UnsafeList)+2*len(u_s)+len(Map.set)+1] = np.finfo(float).eps+1
            
    #         H = np.zeros((len(u_s)+2*len(UnsafeList)+len(Map.set)+1,len(u_s)+2*len(UnsafeList)+len(Map.set)+1))
    #         H[0,0] = 0
    #         H[1,1] = 0

    #         ff = np.zeros((len(u_s)+2*len(UnsafeList)+len(Map.set)+1,1))
    #         for j in range(len(UnsafeList)):
    #                 ff[len(u_s)+2*j] = 65
    #                 H[len(u_s)+2*j+1,len(u_s)+2*j+1] = 10000
    #                 # ff[len(u_s)+2*j+1] = 50* Unsafe.CBF(x_r, x_o[minJ[0][0]][0:2])
    
    #         ff[len(u_s)+2*len(UnsafeList):len(u_s)+2*len(UnsafeList)+len(Map.set)] = 20
    #         ff[-1] = np.ceil(self.count/100.0)
    #     else:
    #         #Ax<=b, x = [v, w , b1, b2,..., bn, b'1, b'2,b'm, delta ]  
    #         # where b is constant in Eq (14) of paper "Risk-bounded  Control  using  Stochastic  Barrier  Functions"
    #         #b' is the slack variable for map constraints 
    #         # delta is for lyapunov function
    #         A = np.zeros((2*len(UnsafeList)+2*len(u_s)+len(Map.set)+2,len(u_s)+len(UnsafeList)+len(Map.set)+1))
    #         b =np.zeros((2*len(u_s)+2*len(UnsafeList)+len(Map.set)+2))
    #         for j in range(len(UnsafeList)):
    #                 # CBF Constraints        
    #                 A[2*j,np.append(np.arange(len(u_s)),[len(u_s)+j])] = [Unsafe.multCond(x_r,  x_o[UnsafeList[j]][0:2],[1, 0]), Unsafe.multCond(x_r,x_o[UnsafeList[j]][0:2],[0, 1]), -1] # multiplier of u , bi
    #                 b[2*j] = -ai* Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2])- Unsafe.ConstCond(x_r,  x_o[UnsafeList[j]][0:2],u_o[UnsafeList[j]]) 
    #                 # Constraints on bi to satisfy pi risk
    #                 A[2*j+1,len(u_s)+j] = 1
    #                 if Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2])<1:
    #                         b[2*j+1] = min(ai, -1/T*log((1-risk)/(1-Unsafe.CBF(x_r, x_o[UnsafeList[j]][0:2]))))
    #                 else:
    #                         b[2*j+1] = 0                        
    #         # Adding U constraint
    #         A[2*len(UnsafeList),0] = 1; b[2*len(UnsafeList)] = U[0,1]
    #         A[2*len(UnsafeList)+1,0] = -1;  b[2*len(UnsafeList)+1] = -U[0,0]
    #         A[2*len(UnsafeList)+2,1] = 1; b[2*len(UnsafeList)+2] = U[1,1]
    #         A[2*len(UnsafeList)+3,1] = -1; b[2*len(UnsafeList)+3] = -U[1,0]
            
    #         # Adding map constraints
    #         for j in range(len(Map.set)):
    #                 A[2*len(UnsafeList)+2*len(u_s)+j,np.append(np.arange(len(u_s)),[len(u_s)+len(UnsafeList)+j])] = [Map.setDer[j](x_r,[1, 0]), Map.setDer[j](x_r,[0, 1]), -1]
    #                 b[2*len(UnsafeList)+2*len(u_s)+j] = -Map.CBF[j](x_r)

    #         # Adding Goal based Lyapunov !!!!!!!!!!!!!!!!! Needs to be changed for a different example 
    #         A[2*len(UnsafeList)+2*len(u_s)+len(Map.set),0:2] = [Goal.Lyap(x_r,[1,0]), Goal.Lyap(x_r,[0, 1])]
    #         A[2*len(UnsafeList)+2*len(u_s)+len(Map.set),-1] = -1
    #         b[2*len(UnsafeList)+2*len(u_s)+len(Map.set)] = 0
    #         A[2*len(UnsafeList)+2*len(u_s)+len(Map.set)+1,-1] = 1
    #         b[2*len(UnsafeList)+2*len(u_s)+len(Map.set)+1] = np.finfo(float).eps+1
            
    #         H = np.zeros((len(u_s)+len(UnsafeList)+len(Map.set)+1,len(u_s)+len(UnsafeList)+len(Map.set)+1))
    #         H[0,0] = 0
    #         H[1,1] = 0

    #         ff = np.zeros((len(u_s)+len(UnsafeList)+len(Map.set)+1,1))
    #         ff[len(u_s):len(u_s)+len(UnsafeList)] = 20
    #         ff[len(u_s)+len(UnsafeList):len(u_s)+len(UnsafeList)+len(Map.set)] = 10
    #         ff[-1] = np.ceil(self.count/100.0)

    #     try:
    #             uq = cvxopt_solve_qp(H, ff, A, b)
    #     except ValueError:
    #             uq = [0,0]
    #             rospy.loginfo('Domain Error in cvx')

    #     if uq is None:
    #             uq = [0,0]
    #             rospy.loginfo('infeasible QP')
        
    #     if findBestCommandAnyway and len(uq[2:len(uq)-2*len(Map.set)-1:2])>0:   # If humans are around and findbestcommand active
    #             if InUnsafe:
    #                     self.trajs.risk.append(1.0)
    #             else:
    #                     r = np.zeros(len(uq[2:len(uq)-2*len(Map.set)-1:2]))
    #                     for k in range(len(uq[2:len(uq)-2*len(Map.set)-1:2])):
    #                             r[k] = min(1, max(0,1-(1-Unsafe.CBF(x_r, x_o[UnsafeList[k]][0:2]))*exp(-uq[2*k+2]*T)))
    #                     self.trajs.risk.append(max(r))    
    #     elif not findBestCommandAnyway and len(uq[2:len(uq)-len(Map.set)-1])>0:
    #             r = np.zeros(len(uq[2:len(uq)-len(Map.set)-1]))
    #             for k in range(len(uq[2:len(uq)-len(Map.set)-1])):
    #                     r[k] = min(1, max(0,1-(1-Unsafe.CBF(x_r, x_o[UnsafeList[k]][0:2]))*exp(-uq[k+2]*T)))
    #             self.trajs.risk.append(max(r))
    #             if max(r)>0.1:
    #                     1
    #     elif not findBestCommandAnyway and len(uq) == 2:  # feasible solution is not found
    #             self.trajs.risk.append(-risk)  # meaning that solution is not found 
    #     else:  # No human is around 
    #             self.trajs.risk.append(0.0)    
    #     self.trajs.minDist.append(minDist)

    #     return uq



    # def controller_loop_callback(self, event):   ## +states
        #         # this controller loop call back.
        #         self.count += 1
        #         now = rospy.get_rostime()
        #         self.trajs.time.append(now.secs+now.nsecs*pow(10,-9))
        #         if DEBUG:
        #                 rospy.loginfo('Current time %i %i', now.secs, now.nsecs)
        #                 rospy.loginfo('tOdometry\n %s', self.odometry)
        #         # get human model state from Gazebo
        #         if self.count==1:
        #                 model_properties = self.get_model_pro()
        #                 for model_name in model_properties.model_names:
        #                         if re.search('actor*', model_name) and not model_name in self.actors:  # if the model name is actor*, it will catch them.
        #                                 self.actors.append(model_name) 
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
        #                 rospy.loginfo('reached Goal set!!')
        #                 rospy.signal_shutdown('reached Goal set')


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
