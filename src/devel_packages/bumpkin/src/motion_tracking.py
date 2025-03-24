#!/usr/bin/env python3
import sys
sys.path.append("/home/ros_ws")
from bumpkin_moveit_class import BKMoveItPlanner
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose
from autolab_core import RigidTransform

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

        # Direction
        current_pose = self.planner.fa.get_pose().translation
        direction = np.array(pose_goal.position) - np.array(current_pose)
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            # Normalize
            direction = direction / distance

            # go slower if its super close to target
            speed_factor = min(1.0, distance / 0.1)  # Full speed beyond 10cm
            step_size = 0.05 * speed_factor  # 5cm max step size
            
            # New position
            new_position = current_pose + direction * step_size

            # Create a RigidTransform object with the position and orientation
            new_pose = RigidTransform(
                translation=new_position,
                rotation=RigidTransform.rotation_from_quaternion([
                    pose_goal.orientation.w,
                    pose_goal.orientation.x,
                    pose_goal.orientation.y,
                    pose_goal.orientation.z
                ])
            )
            
            # Direct control through FrankaArm API
            self.planner.fa.goto_pose(
                new_pose,
                duration=0.1,  # Short duration for responsive movement
                dynamic=True,
                buffer_time=0
            )
            rospy.loginfo(f"DMove to: [{new_position[0]:.3f}, {new_position[1]:.3f}, {new_position[2]:.3f}]")
        
        # # Convert pose to moveit frame
        # pose_goal_moveit = self.planner.get_moveit_pose_given_frankapy_pose(pose_goal)
        
        # # Plan a straight line motion to the goal
        # plan = self.planner.get_straight_plan_given_pose(pose_goal_moveit)

        
        # # Log info
        # rospy.loginfo(f"Received target position: x={msg.x}, y={msg.y}, z={msg.z}")
        # rospy.loginfo("Path planning completed")
        
        # # Execute the plan (uncomment when ready for actual execution)
        # self.planner.execute_plan(plan)

    def shutdown(self):
        rospy.loginfo("Gracefully shutting down")

if __name__ == '__main__':
    #rospy.init_node('motion_tracking_node', anonymous=False)

    ft = FistTracker()
    rospy.on_shutdown(ft.shutdown)

    rospy.spin()