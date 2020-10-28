#! /usr/bin/env python
# call roscore
# $ roscore
#
# If start in manual
# $ rosrun cbf_ros cbf_controller.py

import rospy
import sys
import argparse

# ROS msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

DEBUG = True


class CBF_CONTROLLER(object):
	def __init__(self):
                # publisher to send vw order to HSR
                self.vw_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)

                # subscriber for Gazebo info.
                rospy.wait_for_service ('/gazebo/get_model_state')
                self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                self.tOdometry_subscriber = rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, self.tOdometry_callback, queue_size=10)
                self.odometry = Odometry()

        def __del__(self):
                pass

        def tOdometry_callback(self, odometry):
                self.odometry = odometry

        def controller_loop_callback(self, event):
                # this controller loop call back.

                # just example printf code.
                if DEBUG:
                        now = rospy.get_rostime()
                        rospy.loginfo('Current time %i %i', now.secs, now.nsecs)

                        # true odom
                        tTwist = self.odometry.twist.twist
                        rospy.loginfo('tOdometry \n %s', tTwist)

                # get human model state from Gazebo
                model_actor1 = GetModelStateRequest()
                model_actor1.model_name = 'actor1'
                actor1 = self.get_model_srv(model_actor1)
                if DEBUG:
                        rospy.loginfo('actor1 pose \n %s', actor1.pose)
                
                model_actor2 = GetModelStateRequest()
                model_actor2.model_name = 'actor2'
                actor2 = self.get_model_srv(model_actor2)
                if DEBUG:
                        rospy.loginfo('actor2 pose \n %s', actor2.pose)

                # making vw data and publish it.
                vel_msg = Twist()
                vel_msg.linear.x  = 0.0 #[m/sec]
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
