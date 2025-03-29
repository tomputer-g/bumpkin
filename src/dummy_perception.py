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
        self.rate = rospy.Rate(1) # 1hz

        if len(pts) <= 1:
            rospy.logerr("Need at least 2 points to interpolate traj")
            sys.exit(1)

        for i in range(len(pts)):
            if i == len(pts) - 1:
                end = pts[0]
            else:
                end = pts[i+1]

            move = np.linspace(pts[i], end, num=15) #1hz * 15s legs
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
    pt1 = np.array([0.42, -0.29, 0.58])
    pt2 = np.array([0.47, 0.097, 0.51]) # [-0.02491262  0.83750965  0.030964    0.54497097]
    pt3 = np.array([0.52631634, -0.05520262, 0.3502983 ]) # [ 0.03240049  0.84828503 -0.00749196  0.52849009]
    traj = np.stack([pt1, pt2, pt3])

    temp = GenerateDummyTraj(traj)
    rospy.on_shutdown(temp.shutdown)

    try:
        while not rospy.is_shutdown():
            temp.pub_target()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exiting")