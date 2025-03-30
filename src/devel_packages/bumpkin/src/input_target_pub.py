#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import numpy as np
import sys
import threading
from motion_tracking_franka import FrankaTrajectoryExecutor

class ManualTargetPub(object):
    def __init__(self):
        self.target_pub = rospy.Publisher('/target_pos', Point, queue_size=10)

    def request_input(self):
        i = input("Enter a target position, represented as xyz coordinates separated by spaces: ")
        i = i.split(" ")
        if len(i) != 3:
            rospy.logwarn("Input does not have 3 coordinates. Ignoring")
            return
        
        pt = Point()
        pt.x = float(i[0])
        pt.y = float(i[1])
        pt.z = float(i[2])
        self.target_pub.publish(pt)

    def shutdown(self):
        rospy.loginfo("Gracefully shutting down")

if __name__ == '__main__':
    rospy.init_node('manual_target_pub', anonymous=False)

    target_pub = ManualTargetPub()

    franka_executor = FrankaTrajectoryExecutor()

    rospy.on_shutdown(target_pub.shutdown)
    rospy.on_shutdown(franka_executor.shutdown)

    thread = threading.Thread(target=franka_executor.run)
    thread.daemon = True
    thread.start()

    try:
        while not rospy.is_shutdown():
            target_pub.request_input()
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exiting")