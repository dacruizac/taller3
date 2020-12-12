#!/usr/bin/python

import rospy
from control_prog import control_node

if __name__ == "__main__":
    rospy.init_node("tll3_control",anonymous=True)
    rospy.loginfo("Init node")
    control_node()
    #rospy.spin()