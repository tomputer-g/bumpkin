#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import numpy as np
from frankapy import FrankaArm
from autolab_core import RigidTransform
from scipy.spatial.transform import Rotation
import threading
from collections import deque

class FrankaTrajectoryExecutor:
    def __init__(self):
        self.rate = rospy.Rate(10)  # Increase control loop frequency to 10Hz
        
        # Initialize
        rospy.loginfo("Initializing FrankaArm...")
        self.fa = FrankaArm(init_node=False, with_gripper=False)

        # Go to home pose 
        rospy.loginfo("Going to initial pose...")

        # EE orientation
        self.default_orientation = np.array([[4.80390274e-01, -4.80102472e-01,  7.33980109e-01],
                                        [-7.06876315e-01, -7.07337172e-01, -2.42752946e-05],
                                        [5.19183069e-01, -5.18821493e-01, -6.79170964e-01]])

        transform = RigidTransform(
                rotation=self.default_orientation,
                translation=self.fa.get_pose().translation,
                from_frame='franka_tool',
                to_frame='world'
            )

        self.fa.goto_pose(
            transform,
            use_impedance=True,
            cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
        )

        # Use a deque to store waypoints
        self.waypoint_queue = deque(maxlen=100)
        self.waypoint_lock = threading.Lock()
        
        # State tracking
        self.current_target = None
        self.is_moving = False
        self.motion_thread = None
        
        # Subscribe to target position topic
        self.target_sub = rospy.Subscriber('/target_pos', Point, self.target_callback)
        
        # Safety limit distance
        self.max_position_change = 0.10  # 10cm
        self.min_waypoint_distance = 0.01  # 1cm
        self.last_published_target = None
        
        rospy.loginfo("Franka ready.")      
    
    def target_callback(self, msg):
        # Callback for receiving new target positions
        target_position = np.array([msg.x, msg.y, msg.z])
        
        # Get current position
        current_position = self.fa.get_pose().translation
        
        # Safety check - limit maximum position change from current position
        distance_to_target = np.linalg.norm(target_position - current_position)
        
        if distance_to_target > self.max_position_change:
            rospy.logwarn(f"Target position too far ({distance_to_target:.2f}m). Limiting movement to {self.max_position_change}m.")
            direction = (target_position - current_position) / distance_to_target
            target_position = current_position + direction * self.max_position_change
        
        # If we have a previous target, check if this one is significantly different
        if self.last_published_target is not None:
            distance_from_last = np.linalg.norm(target_position - self.last_published_target)
            
            # If the target hasn't changed significantly, ignore
            if distance_from_last < self.min_waypoint_distance:  # 1cm threshold
                return
        
        # Update last published target
        self.last_published_target = target_position
        
        # Add waypoint to queue with thread safety
        with self.waypoint_lock:
            self.waypoint_queue.append(target_position)
            
        rospy.loginfo(f"New target received: [{target_position[0]:.4f}, {target_position[1]:.4f}, {target_position[2]:.4f}]")
        
        # Start motion thread if not already running
        if not self.is_moving:
            self.start_motion_thread()
    
    def start_motion_thread(self):
        if self.is_moving:
            return
            
        self.is_moving = True
        self.motion_thread = threading.Thread(target=self.continuous_motion_loop)
        self.motion_thread.daemon = True
        self.motion_thread.start()
    
    def continuous_motion_loop(self):
        """Main motion loop that continuously processes waypoints"""
        try:
            while not rospy.is_shutdown():
                # Check if we have waypoints to process
                if len(self.waypoint_queue) == 0:
                    rospy.loginfo("No more waypoints, stopping continuous motion")
                    self.is_moving = False
                    return
                
                # Get current position and next waypoint
                current_position = self.fa.get_pose().translation
                
                # Thread-safe access to waypoint queue
                with self.waypoint_lock:
                    next_waypoint = self.waypoint_queue.popleft()
                
                distance = np.linalg.norm(next_waypoint - current_position)
                
                # Calculate appropriate duration and impedance based on distance
                base_duration = 0.3  # reduced minimum duration for more responsiveness
                distance_factor = 1.5  # seconds per meter of travel
                duration = base_duration + distance * distance_factor
                duration = min(duration, 2.0)  # cap at 2 seconds max
                
                # Adjust impedance for smoother motion at different speeds
                pos_impedance = max(300.0, min(600.0, 600.0 - distance * 1000))
                
                # Create transform for the waypoint
                target_transform = RigidTransform(
                    rotation=self.default_orientation,
                    translation=next_waypoint,
                    from_frame='franka_tool',
                    to_frame='world'
                )
                
                rospy.loginfo(f"Moving to waypoint: {next_waypoint} (distance: {distance:.3f}m, duration: {duration:.2f}s)")
                
                # Execute motion to waypoint
                self.fa.goto_pose(
                    target_transform,
                    duration=float(duration),
                    use_impedance=True,
                    cartesian_impedances=[pos_impedance, pos_impedance, pos_impedance, 50.0, 50.0, 50.0]
                )
                
                # Check if we received any new waypoints during execution
                # If not, we'll exit the loop on the next iteration
                if len(self.waypoint_queue) == 0:
                    self.rate.sleep()  # Give a small delay before exiting
                
        except Exception as e:
            rospy.logerr(f"Error in continuous motion loop: {str(e)}")
        
        self.is_moving = False
    
    def run(self):
        while not rospy.is_shutdown():
            # Main control loop now just monitors and handles any errors
            # The actual motion happens in the continuous_motion_loop thread
            self.rate.sleep()
    
    def shutdown(self):
        rospy.loginfo("Shutting down Franka trajectory executor...")
        try:
            self.fa.stop_skill()
            self.fa.reset_joints()
        except:
            rospy.logwarn("Could not reset to home position during shutdown")
        rospy.loginfo("Shutdown complete")

if __name__ == '__main__':
    try:
        rospy.init_node('motion_track', anonymous=False)
        executor = FrankaTrajectoryExecutor()
        rospy.on_shutdown(executor.shutdown)
        
        executor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received, shutting down")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {str(e)}")