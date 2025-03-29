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

        self.latest_target = None
        self.is_executing = False

        self.target_sub = rospy.Subscriber('/target_pos', Point, self.update_target, queue_size=1)

        self.tracking_timer = rospy.Timer(rospy.Duration(0.5), self.track)

    def update_target(self, msg):
        self.latest_target = msg
        rospy.loginfo(f"Received target position: x={msg.x}, y={msg.y}, z={msg.z}")

    def track(self, msg):
        # convert target ee point to joint positions src/devel_packages/manipulation/src/moveit_class.py
        # plan and execute path to target 

        if self.is_executing or not self.latest_target:
            return
        
        try:
            self.is_executing = True
            target = copy.deepcopy(self.latest_target)
        
            # Create a pose goal using the received point
            pose_goal = Pose()
            pose_goal.position.x = target.x
            pose_goal.position.y = target.y
            pose_goal.position.z = target.z
            
            # Using fixed orientation values of ee (change this)
            pose_goal.orientation.x = 0.01039112
            pose_goal.orientation.y = 0.80840715
            pose_goal.orientation.z = 0.00958253
            pose_goal.orientation.w = 0.58844988
        
            # Convert pose to moveit frame
            pose_goal_moveit = self.planner.get_moveit_pose_given_frankapy_pose(pose_goal)
            
            # Plan a straight line motion to the goal
            plan = self.planner.get_straight_plan_given_pose(pose_goal_moveit)
            
            # Log info
            rospy.loginfo("Path planning completed")
        
            # Execute the plan (uncomment when ready for actual execution)
            self.planner.execute_plan(plan)
            rospy.loginfo("Execution completed")

        except Exception as e:
            rospy.logerr(f"Error in tracking: {e}")

        finally:
            self.is_executing = False

    def shutdown(self):
        self.tracking_timer.shutdown()
        rospy.loginfo("Gracefully shutting down")

if __name__ == '__main__':
    # rospy.init_node('motion_tracking_node', anonymous=False)

    ft = FistTracker()
    rospy.on_shutdown(ft.shutdown)

    rospy.spin()