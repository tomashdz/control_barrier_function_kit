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

DEBUG = False


class CBF_CONTROLLER(object):
	def __init__(self):
                # publisher to send vw order to HSR
                self.vw_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)


        def __del__(self):
                pass


        def controller_loop_callback(self, event):
                now = rospy.get_rostime()
                rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

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
                control_priod = 0.1 #[sec]
                rospy.Timer(rospy.Duration(control_priod), cbf_controller.controller_loop_callback)
                rospy.spin()

        except rospy.ROSInterruptException:
                pass
