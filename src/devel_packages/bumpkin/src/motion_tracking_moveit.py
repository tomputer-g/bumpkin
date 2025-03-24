#!/usr/bin/env python3
import sys
sys.path.append("/home/ros_ws")
from bumpkin_moveit_class import BKMoveItPlanner

import rospy
from geometry_msgs.msg import Point, Pose
import numpy as np
import threading

class MoveItTracker(object):
    def __init__(self):
        # Initialize the MoveIt planner
        self.planner = BKMoveItPlanner()
        
        # Tracking parameters
        self.target_position = None
        self.last_planned_target = None
        self.is_executing = False
        self.target_lock = threading.Lock()
        self.update_threshold = 0.05  # Only replan if target moves more than 5cm
        
        # Default orientation - modify as needed for your use case
        self.default_orientation = Pose()
        self.default_orientation.orientation.x = 0.01039112
        self.default_orientation.orientation.y = 0.80840715
        self.default_orientation.orientation.z = 0.00958253
        self.default_orientation.orientation.w = 0.58844988
        
        # Subscribe to target positions
        self.target_sub = rospy.Subscriber('/target_pos', Point, self.update_target, queue_size=1)
        
        # Start tracking thread
        self.tracking_thread = threading.Thread(target=self.tracking_loop)
        self.tracking_thread.daemon = True
        self.tracking_thread.start()
        
        rospy.loginfo("MoveIt tracker initialized and ready")
    
    def update_target(self, msg):
        """Callback to update the target position"""
        with self.target_lock:
            self.target_position = [msg.x, msg.y, msg.z]
        rospy.loginfo(f"Received target position: x={msg.x}, y={msg.y}, z={msg.z}")
    
    def tracking_loop(self):
        """Main tracking loop"""
        rate = rospy.Rate(1)  # Check for new targets at 1Hz
        
        while not rospy.is_shutdown():
            try:
                # Check if we have a target and if it's different from the last one we planned for
                should_plan = False
                current_target = None
                
                with self.target_lock:
                    if self.target_position is not None:
                        current_target = self.target_position.copy()
                        
                        # Check if we need to plan a new trajectory
                        if self.last_planned_target is None:
                            should_plan = True
                        elif not self.is_executing:
                            # Only check distance if we're not currently executing a trajectory
                            distance = np.linalg.norm(np.array(current_target) - np.array(self.last_planned_target))
                            should_plan = distance > self.update_threshold
                
                if should_plan and current_target is not None:
                    self.plan_and_execute(current_target)
                    with self.target_lock:
                        self.last_planned_target = current_target
            
            except Exception as e:
                rospy.logerr(f"Error in tracking loop: {e}")
            
            rate.sleep()
    
    def plan_and_execute(self, target):
        """Plan and execute a trajectory to the target"""
        try:
            # Set executing flag
            self.is_executing = True
            
            # Create pose goal
            pose_goal = Pose()
            pose_goal.position.x = target[0]
            pose_goal.position.y = target[1]
            pose_goal.position.z = target[2]
            pose_goal.orientation = self.default_orientation.orientation
            
            # Convert to MoveIt frame
            moveit_pose = self.planner.get_moveit_pose_given_frankapy_pose(pose_goal)
            
            # Plan a trajectory - first try with straight line planning
            plan = self.planner.get_straight_plan_given_pose(moveit_pose)
            
            # If straight plan fails or is empty, try regular planning
            if len(plan) == 0:
                rospy.logwarn("Straight line planning failed, trying regular planning")
                plan = self.planner.get_plan_given_pose(moveit_pose)
            
            # Execute the plan if valid
            if len(plan) > 0:
                rospy.loginfo(f"Executing path with {len(plan)} waypoints")
                self.planner.execute_plan(plan)
            else:
                rospy.logwarn("Could not plan a valid path to target")
                
        except Exception as e:
            rospy.logerr(f"Planning/execution error: {e}")
        finally:
            # Reset executing flag
            self.is_executing = False
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Gracefully shutting down tracker")
        self.planner.fa.stop_skill()

if __name__ == '__main__':
    try:
        tracker = MoveItTracker()
        rospy.on_shutdown(tracker.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass