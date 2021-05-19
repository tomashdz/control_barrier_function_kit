from sympy import symbols, Matrix, sin, cos, lambdify, exp, sqrt, log, diff
import numpy as np


class CBF(object):
    def __init__(self, h, BF, ego, agent):
        self.h = h
        self.BF = BF
        self.states = [ego.states, agent.states]
        self.compute_LHS_RHS(ego, agent)
        
        # self.B = exp(-gamma*Uset)


    def compute_LHS_RHS(self, ego , agent):
        """
        B_dot <= -alpha*B(x) if B<=0 is safe
        LHS*inputs <= RHS

        Args:
            agent ([type]): [description]
        """
        alpha = 1
        # h_inp = [ego.states, agent.states]
        BFsym = self.BF(*self.states)
        BF_d = BFsym.diff(Matrix([ego.states,agent.states]))
        self.LHS = lambdify([ego.states,agent.states], -alpha*BFsym-(BF_d.T*Matrix([ego.f,agent.f]))[0])
        self.RHS = lambdify([ego.states,agent.states], (Matrix(BF_d[:ego.nDim]).T*ego.g)[0])

        # BF_d2 =  self.BF.diff(self.x_o_s,2)
        # UnsafeInfo.CBF = lambdify([self.x_r_s,self.x_o_s], CBF)

    def details(self):
        return '{}\n {}\n {}\n'.format(self.h(*self.states), self.BF(*self.states), self.states)
    # def add_constraint(self):
    #     pass
    # def remove_constraint(self):
    #     pass

class Control_CBF(object):
    def __init__(ego, agent_CBF, goal_set_func, P = None, Q = None ):

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



        for j in agent_CBF:
            if type(j) is not tuple:
                raise ValueError("second argument should be a list of tuples") 

        self.ego = ego
        self.agent_CBF = agent_CBF
        self.goal_set_func = goal_set_func
        self.P = 0
        self.Q = 0
        
        
        def cbf_controller_compute(self,state):
            pass


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



        #         u = self.cbf_controller_compute()
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
