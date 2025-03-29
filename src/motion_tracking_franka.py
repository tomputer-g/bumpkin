# #!/usr/bin/env python3
# import sys
# sys.path.append("/home/ros_ws")
# from src.devel_packages.manipulation.src.moveit_class import MoveItPlanner

# import rospy
# from geometry_msgs.msg import Point, Pose
# import copy

# class FistTracker(object):
#     def __init__(self):
#         self.planner = MoveItPlanner()

#         self.latest_target = None
#         self.is_executing = False

#         self.target_sub = rospy.Subscriber('/target_pos', Point, self.update_target, queue_size=1)

#         self.tracking_timer = rospy.Timer(rospy.Duration(0.5), self.track)

#     def update_target(self, msg):
#         self.latest_target = msg
#         rospy.loginfo(f"Received target position: x={msg.x}, y={msg.y}, z={msg.z}")

#     def track(self, msg):
#         # convert target ee point to joint positions src/devel_packages/manipulation/src/moveit_class.py
#         # plan and execute path to target 

#         if self.is_executing or not self.latest_target:
#             return
        
#         try:
#             self.is_executing = True
#             target = copy.deepcopy(self.latest_target)
        
#             # Create a pose goal using the received point
#             pose_goal = Pose()
#             pose_goal.position.x = target.x
#             pose_goal.position.y = target.y
#             pose_goal.position.z = target.z
            
#             # Using fixed orientation values of ee (change this)
#             pose_goal.orientation.x = 0.01039112
#             pose_goal.orientation.y = 0.80840715
#             pose_goal.orientation.z = 0.00958253
#             pose_goal.orientation.w = 0.58844988
        
#             # Convert pose to moveit frame
#             pose_goal_moveit = self.planner.get_moveit_pose_given_frankapy_pose(pose_goal)
            
#             # Plan a straight line motion to the goal
#             plan = self.planner.get_straight_plan_given_pose(pose_goal_moveit)
            
#             # Log info
#             rospy.loginfo("Path planning completed")
        
#             # Execute the plan (uncomment when ready for actual execution)
#             self.planner.execute_plan(plan)
#             rospy.loginfo("Execution completed")

#         except Exception as e:
#             rospy.logerr(f"Error in tracking: {e}")

#         finally:
#             self.is_executing = False

#     def shutdown(self):
#         self.tracking_timer.shutdown()
#         rospy.loginfo("Gracefully shutting down")

# if __name__ == '__main__':
#     # rospy.init_node('motion_tracking_node', anonymous=False)

#     ft = FistTracker()
#     rospy.on_shutdown(ft.shutdown)

#     rospy.spin()

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import numpy as np
from frankapy import FrankaArm
from frankapy import FrankaPyDatasetLogger
from frankapy.utils import convert_rigid_transform_to_array
import time
from autolab_core import RigidTransform
from scipy.spatial.transform import Rotation

class FrankaTrajectoryExecutor:
    def __init__(self):
        rospy.init_node('franka_trajectory_executor', anonymous=False)
        self.rate = rospy.Rate(10)  # 10Hz control loop
        
        # Initialize FrankaArm
        rospy.loginfo("Initializing FrankaArm...")
        self.fa = FrankaArm()
        
        # Reset Franka to home position
        rospy.loginfo("Homing FrankaArm...")
        self.fa.reset_joints()
        
        # Current target and execution status
        self.current_target = None
        self.new_target_received = False
        self.executing_trajectory = False
        
        # Get current end-effector orientation and use it throughout the task
        self.current_pose = self.fa.get_pose()
        self.default_orientation = self.current_pose.rotation
        
        # Subscribe to target position topic
        self.target_sub = rospy.Subscriber('/target_pos', Point, self.target_callback)
        
        # Set up a safety limit for movement (in meters)
        self.max_position_change = 0.1  # 10cm
        
        rospy.loginfo("Franka trajectory executor initialized and ready.")
    
    def target_callback(self, msg):
        """Callback for receiving new target positions"""
        # Skip if we're currently executing a trajectory
        if self.executing_trajectory:
            rospy.logwarn("Received new target while executing trajectory. Will process after completion.")
            return
        
        # Extract position from message
        target_position = np.array([msg.x, msg.y, msg.z])
        
        # Check if this is a new target to avoid unnecessary movements
        if self.current_target is not None:
            # Calculate distance to new target
            distance = np.linalg.norm(target_position - self.current_target)
            
            # If the target hasn't changed significantly, ignore it
            if distance < 0.01:  # 1cm threshold
                return
        
        # Safety check - limit maximum position change
        current_position = self.fa.get_pose().translation
        distance_to_target = np.linalg.norm(target_position - current_position)
        
        if distance_to_target > self.max_position_change:
            rospy.logwarn(f"Target position too far ({distance_to_target:.2f}m). Limiting movement to {self.max_position_change}m.")
            direction = (target_position - current_position) / distance_to_target
            target_position = current_position + direction * self.max_position_change
        
        self.current_target = target_position
        self.new_target_received = True
        rospy.loginfo(f"New target received: [{target_position[0]:.4f}, {target_position[1]:.4f}, {target_position[2]:.4f}]")
    
    def execute_trajectory(self):
        """Execute trajectory to the current target"""
        if not self.new_target_received:
            return
        
        self.executing_trajectory = True
        self.new_target_received = False
        
        try:
            # Create a rigid transform with current orientation and new position
            target_transform = RigidTransform(
                rotation=self.default_orientation,
                translation=self.current_target,
                from_frame='franka_tool',
                to_frame='world'
            )
            
            rospy.loginfo(f"Moving to target: {self.current_target}")
            
            # Execute trajectory with reasonable speed
            duration = 2.0  # seconds
            self.fa.goto_pose(
                target_transform,
                duration=duration,
                use_impedance=True,
                cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
            )
            
            # Small delay to ensure completion
            rospy.sleep(0.1)
            
            rospy.loginfo(f"Reached target position")
            
        except Exception as e:
            rospy.logerr(f"Error executing trajectory: {str(e)}")
        
        self.executing_trajectory = False
    
    def run(self):
        """Main control loop"""
        while not rospy.is_shutdown():
            if self.new_target_received and not self.executing_trajectory:
                self.execute_trajectory()
            self.rate.sleep()
    
    def shutdown(self):
        """Graceful shutdown handler"""
        rospy.loginfo("Shutting down Franka trajectory executor...")
        # Return to a safe position before shutting down
        try:
            self.fa.stop_skill()
            self.fa.reset_joints()
        except:
            rospy.logwarn("Could not reset to home position during shutdown")
        rospy.loginfo("Shutdown complete")

if __name__ == '__main__':
    try:
        executor = FrankaTrajectoryExecutor()
        rospy.on_shutdown(executor.shutdown)
        executor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received, shutting down")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {str(e)}")