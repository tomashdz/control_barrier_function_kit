#! /usr/bin/env python
# call roscore
# $ roscore
#
# If start in manual
# $ rosrun cbf_ros cbf_controller.py
#
# About coordination
# HSR is handled by \odom coordination and Gazebo is handled by \map coordination 
# \odom: That is robots global position. Usually, the center of coodination means when robot is started. The coordination will have localizaion error and updated by any loclaiozer frequently.
# \map: That is map's center, like center of Earth. Generally robot will not start from the center. 
# \base_foot_print: the robot center coordination. We can build controller based on this coordinaiton. 

import rospy
import sys
import argparse

# ROS msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

# ROS others
import tf

DEBUG = True

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

class CBF_CONTROLLER(object):
	def __init__(self):
                # publisher to send vw order to HSR
                self.vw_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)

                # subscriber for Gazebo info.
                rospy.wait_for_service ('/gazebo/get_model_state')
                self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                self.tOdometry_subscriber = rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, self.tOdometry_callback, queue_size=10)
                self.tOdometry = Odometry()
                self.odometry_subscriber = rospy.Subscriber('/global_pose', PoseStamped, self.odometry_callback, queue_size=10)
                self.poseStamped = PoseStamped()

                # listener of tf.
                self.tfListener = tf.TransformListener()

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

                # just example printf code.
                if DEBUG:
                        now = rospy.get_rostime()
                        rospy.loginfo('Current time %i %i', now.secs, now.nsecs)

                        # true odom
                        rospy.loginfo('tOdometry\n %s', self.odometry)

                        # odom
                        #rospy.loginfo('odometry \n %s', self.poseStamped)

                # covert todometry 

                # get human model state from Gazebo
                model_actor1 = GetModelStateRequest()
                model_actor1.model_name = 'actor1'
                actor1 = self.get_model_srv(model_actor1) # the pose date is based on /map
                actor1base_footprint_pose = self.gazebo_pos_transformPose('base_footprint', actor1) # trasfer /map->/base_footprint
                angular1 = orientation2angular(actor1base_footprint_pose.pose.orientation)      # transfer orientaton(quaternion)->agular(euler)
                if DEBUG:
                        rospy.loginfo('actor1 in base_footprint\nposition:\n%s\nangular:\n%s', actor1base_footprint_pose.pose.position, angular1)

                model_actor2 = GetModelStateRequest()
                model_actor2.model_name = 'actor2'
                actor2 = self.get_model_srv(model_actor2) # the pose date is based on /map
                actor2base_footprint_pose = self.gazebo_pos_transformPose('base_footprint', actor2) # trasfer /map->/base_footprint
                angular2 = orientation2angular(actor2base_footprint_pose.pose.orientation)      # transfer orientaton(quaternion)->agular(euler)
                if DEBUG:
                        rospy.loginfo('actor2 in base_footprint\nposition:\n%s\nangular:\n%s', actor2base_footprint_pose.pose.position, angular2)

                # making vw data and publish it.
                vel_msg = Twist()
                vel_msg.linear.x  = 0.3 #[m/sec]
                vel_msg.angular.z = 0.0 #[rad/sec]
                self.vw_publisher.publish(vel_msg)


if __name__ == '__main__':
        # Process arguments
        p = argparse.ArgumentParser(description='CBF controller')
        args = p.parse_args(rospy.myargv()[1:])

        try:
	        rospy.init_node('cbf_controller')
                cbf_controller = CBF_CONTROLLER()
                control_priod = 0.1 #[sec] we can change controll priod with this parameter.
                rospy.Timer(rospy.Duration(control_priod), cbf_controller.controller_loop_callback)
                rospy.spin()

        except rospy.ROSInterruptException:
                pass
