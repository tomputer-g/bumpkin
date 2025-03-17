#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import numpy as np
import sys

class GenerateDummyTraj(object):
    def __init__(self, pts):
        self.traj = pts[0]
        self.i = 0
        self.dir = 1
        self.target_pub = rospy.Publisher('/target_pos', Point, queue_size=10)
        self.rate = rospy.Rate(5) # 5hz

        if len(pts) <= 1:
            rospy.logerr("Need at least 2 points to interpolate traj")
            sys.exit(1)

        for i in range(len(pts)):
            if i == len(pts) - 1:
                end = pts[0]
            else:
                end = pts[i+1]

            move = np.linspace(pts[i], end, num=75) #5hz * 15s legs
            wait = np.stack([end] * 25)
            leg = np.vstack([move, wait])
            self.traj = np.vstack([self.traj, leg])

        #rospy.loginfo(self.traj.shape)

    def pub_target(self):
        arr = self.traj[self.i]
        msg = Point()
        msg.x = arr[0]
        msg.y = arr[1]
        msg.z = arr[2]
        #rospy.loginfo(f"{arr}")
        self.target_pub.publish(msg)

        if self.i == len(self.traj) - 1 and self.dir == 1:
            self.dir = -1
        elif self.i == 0 and self.dir == -1:
            self.dir = 1
        self.i += self.dir
        self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Gracefully shutting down")

if __name__ == '__main__':
    rospy.init_node('dummy_perception_node', anonymous=False)
    pt1 = np.array([0, 0.1, 0.2])
    pt2 = np.array([0.5, 0.2, 0.2])
    pt3 = np.array([-0.5, 0.3, 0.2])
    traj = np.stack([pt1, pt2, pt3])

    temp = GenerateDummyTraj(traj)
    rospy.on_shutdown(temp.shutdown)

    try:
        while not rospy.is_shutdown():
            temp.pub_target()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exiting")