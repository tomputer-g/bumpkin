#!/usr/bin/env python3
import sys
sys.path.append("/home/ros_ws")
from src.devel_packages.manipulation.src.moveit_class import MoveItPlanner

import rospy
from geometry_msgs.msg import Point, Pose

class FistTracker(object):
    def __init__(self):
        self.planner = MoveItPlanner()

        self.target_sub = rospy.Subscriber('/target_pos', Point, self.track, queue_size=1)

    def track(self, msg):
        # convert target ee point to joint positions 
        # plan and execute path to target 
        
        # Create a pose goal using the received point
        # pose_goal = Pose()
        # pose_goal.position.x = msg.x
        # pose_goal.position.y = msg.y
        # pose_goal.position.z = msg.z
        
        # # Using fixed orientation values of ee (change this)
        # pose_goal.orientation.x = -0.9186984147774666
        # pose_goal.orientation.y = 0.3942492534293267
        # pose_goal.orientation.z = -0.012441904611284204 
        # pose_goal.orientation.w = 0.020126567105018894
        
        # # Access the global MoveItPlanner instance
        # franka_moveit = MoveItPlanner()
        
        # # Convert pose to moveit frame
        # pose_goal_moveit = franka_moveit.get_moveit_pose_given_frankapy_pose(pose_goal)
        
        # # Plan a straight line motion to the goal
        # plan = franka_moveit.get_straight_plan_given_pose(pose_goal_moveit)
        
        # Log info
        rospy.loginfo(f"Received target position: x={msg.x}, y={msg.y}, z={msg.z}")
        rospy.loginfo("Path planning completed")
        
        # Execute the plan (uncomment when ready for actual execution)
        # franka_moveit.execute_plan(plan)

    def shutdown(self):
        rospy.loginfo("Gracefully shutting down")

if __name__ == '__main__':
    #rospy.init_node('motion_tracking_node', anonymous=False)

    ft = FistTracker()
    rospy.on_shutdown(ft.shutdown)

    rospy.spin()