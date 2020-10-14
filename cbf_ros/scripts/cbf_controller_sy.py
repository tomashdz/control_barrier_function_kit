#! /usr/bin/env python
# call roscore
# $ roscore
#
# If start in manual
# $ rosrun cbf_ros cbf_controller.py

import rospy
import sys
import argparse
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ROS msg
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

DEBUG = False


class CBF_CONTROLLER(object):
	def __init__(self):
                # publisher to send vw order to HSR
                self.vw_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)

                # subscriber for Gazebo info.
                rospy.wait_for_service ('/gazebo/get_model_state')
                self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                self.flag = 0

        def __del__(self):
                pass


        def controller_loop_callback(self, event):
                # this controller loop call back.

                # just example printf code.
                if DEBUG:
                        now = rospy.get_rostime()
                        rospy.loginfo('Current time %i %i', now.secs, now.nsecs)
                # get hsr state from Gazebo
                model_hsr = GetModelStateRequest()
                model_hsr.model_name = 'hsrb'
                hsrb = self.get_model_srv(model_hsr)
                if DEBUG:
                        rospy.loginfo(hsrb)
                        rospy.loginfo("hsr pose \n %s", hsrb.pose) 
                # get human model state from Gazebo
                model_actor1 = GetModelStateRequest()
                model_actor1.model_name = 'actor1'
                actor1 = self.get_model_srv(model_actor1)
                if DEBUG:
                        # rospy.loginfo(actor1)
                        # rospy.loginfo("actor1 pose \n %s", actor1.pose)
                
                model_actor2 = GetModelStateRequest()
                model_actor2.model_name = 'actor2'
                actor2 = self.get_model_srv(model_actor2)
                if DEBUG:
                        # rospy.loginfo(actor2)
                        #rospy.loginfo("actor2 pose \n %s", actor2.pose)

                # making vw data and publish it.
                vel_msg = Twist()
                orientation_q = hsrb.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
                # print yaw
        
                if abs(hsrb.pose.position.x)>.1 and self.flag==0:
                        vel_msg.linear.x  = -0.3*(hsrb.pose.position.x)/abs(hsrb.pose.position.x) #[m/sec]
                        vel_msg.angular.z = -10*hsrb.pose.orientation.z 
                        # rospy.loginfo("branch 1")
                elif abs(actor2.pose.position.x)<2 and hsrb.pose.position.y<actor2.pose.position.y:
                         vel_msg.linear.x  = 0
                         vel_msg.angular.z = np.pi/2-yaw
                        #  rospy.loginfo("here in 2")
                else:
                         vel_msg.linear.x  = 0.3
                         vel_msg.angular.z = np.pi/2-yaw
                         self.flag = 1
                        #  rospy.loginfo("z orientation = %s", hsrb.pose.orientation.z/(np.pi/4))
        
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
