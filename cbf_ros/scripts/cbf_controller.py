#! /usr/bin/env python
# call roscore
# $ roscore
#
# If start in manual
# $ rosrun cbf_ros cbf_controller.py

import rospy
import sys
import argparse

DEBUG = False


class CBF_CONTROLLER(object):
	def __init__(self):
                pass


        def __del__(self):
                pass


        def controller_loop_callback(self, event):
                now = rospy.get_rostime()
                rospy.loginfo("Current time %i %i", now.secs, now.nsecs)


if __name__ == '__main__':
        # Process arguments
        p = argparse.ArgumentParser(description='CBF controller')
        args = p.parse_args(rospy.myargv()[1:])

        try:
	        rospy.init_node('cbf_controller')
                cbf_controller = CBF_CONTROLLER()
                rospy.Timer(rospy.Duration(0.1), cbf_controller.controller_loop_callback)
                rospy.spin()

        except rospy.ROSInterruptException:
                pass
