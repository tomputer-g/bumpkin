#!/usr/bin/env python3
import sys
sys.path.append("/home/ros_ws")
from bumpkin_moveit_class import BKMoveItPlanner

import rospy
from geometry_msgs.msg import Point, Pose
import copy

class FistTracker(object):
    def __init__(self):
        self.planner = BKMoveItPlanner()

        self.target_sub = rospy.Subscriber('/target_pos', Point, self.track, queue_size=1)

    def track(self, msg):
        # convert target ee point to joint positions
        # plan and execute path to target 
        
        # Create a pose goal using the received point
        pose_goal = Pose()
        pose_goal.position.x = msg.x
        pose_goal.position.y = msg.y
        pose_goal.position.z = msg.z
        
        # Using fixed orientation values of ee (change this)
        pose_goal.orientation.x = 0.01039112
        pose_goal.orientation.y = 0.80840715
        pose_goal.orientation.z = 0.00958253
        pose_goal.orientation.w = 0.58844988
        
        # Convert pose to moveit frame
        pose_goal_moveit = self.planner.get_moveit_pose_given_frankapy_pose(pose_goal)
        
        # Plan a straight line motion to the goal
        # plan = self.planner.get_straight_plan_given_pose(pose_goal_moveit)

        waypoints = []
        waypoints.append(copy.deepcopy(pose_goal_moveit))
        (plan, fraction) = self.planner.group.compute_cartesian_path(
            waypoints, 0.01, 0  # waypoints to follow  # eef_step
        )  # jump_threshold
        self.planner.group.execute(plan, wait=True)
        
        # Log info
        rospy.loginfo(f"Received target position: x={msg.x}, y={msg.y}, z={msg.z}")
        rospy.loginfo("Path planning completed")
        
        # Execute the plan (uncomment when ready for actual execution)
        # self.planner.execute_plan(plan)

    def shutdown(self):
        rospy.loginfo("Gracefully shutting down")

if __name__ == '__main__':
    #rospy.init_node('motion_tracking_node', anonymous=False)

    ft = FistTracker()
    rospy.on_shutdown(ft.shutdown)

    rospy.spin()