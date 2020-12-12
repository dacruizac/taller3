#!/usr/bin/python

import rospy
from graph_prog import planning_controller

if __name__ == "__main__":
    rospy.init_node("tll3_planing",anonymous=True)
    rospy.loginfo("Init node")
    planning_controller()
    #rospy.spin()