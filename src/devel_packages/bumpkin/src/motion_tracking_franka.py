#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import numpy as np
from frankapy import FrankaArm
from autolab_core import RigidTransform
from scipy.spatial.transform import Rotation

class FrankaTrajectoryExecutor:
    def __init__(self):

        self.rate = rospy.Rate(1000)  # 1000Hz control loop
        
        # Initialize
        rospy.loginfo("Initializing FrankaArm...")
        self.fa = FrankaArm(init_node=False, with_gripper=False)

        # Go to home pose 
        rospy.loginfo("Going to fist bump pose...")

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

        self.current_target = None
        self.new_target_received = False
        self.executing_trajectory = False
        self.current_pose = self.fa.get_pose()
        
        self.target_sub = rospy.Subscriber('/target_pos', Point, self.target_callback)
        
        # Safety limit distance
        self.max_position_change = 0.10  # 40cm
        
        rospy.loginfo("Franka ready.")      
    
    def target_callback(self, msg):
        # Callback for receiving new target positions
 
        if self.executing_trajectory:
            rospy.logwarn("Received new target while executing trajectory, skip")
            return
        
        target_position = np.array([msg.x, msg.y, msg.z])
        
        if self.current_target is not None:

            distance = np.linalg.norm(target_position - self.current_target)
            
            # If the target hasn't changed significantly, ignore
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
        if not self.new_target_received:
            return
        
        self.executing_trajectory = True
        self.new_target_received = False
        
        try:
            current_position = self.fa.get_pose().translation
            target_position = self.current_target
            
            distance = np.linalg.norm(target_position - current_position)
            
            base_duration = 1.5  # minimum duration in seconds
            distance_factor = 2.0  # seconds per meter of travel
            duration = base_duration + distance * distance_factor
            
            duration = min(duration, 4.0)
            
            rospy.loginfo(f"Moving to target: {self.current_target} (distance: {distance:.3f}m, duration: {duration:.2f}s)")
            
            # For longer movements, use intermediate waypoints
            if distance > 0.15: 
                rospy.loginfo("Using intermediate waypoints for smoother motion")
                
                num_waypoints = max(2, int(distance * 10))  # More waypoints for longer distances
                
                # Use cubic interpolation for position (smoother acceleration/deceleration)
                t_values = np.linspace(0, 1, num_waypoints)
                
                # Apply cubic ease-in/ease-out function
                # This creates a smoother acceleration and deceleration profile
                t_smooth = t_values**2 * (3 - 2 * t_values)
                
                # Interpolate positions
                waypoints = []
                for t in t_smooth:
                    pos = current_position + t * (target_position - current_position)
                    transform = RigidTransform(
                        rotation=self.default_orientation,
                        translation=pos,
                        from_frame='franka_tool',
                        to_frame='world'
                    )
                    waypoints.append((transform, duration / num_waypoints))

                for i, (waypoint, waypoint_duration) in enumerate(waypoints):
                    rospy.loginfo(f"Executing waypoint {i+1}/{len(waypoints)}")
                    self.fa.goto_pose(
                        waypoint,
                        duration=float(waypoint_duration),
                        use_impedance=True,
                        cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
                    )
            else:
                # For shorter movements, just go to pose
                target_transform = RigidTransform(
                    rotation=self.default_orientation,
                    translation=target_position,
                    from_frame='franka_tool',
                    to_frame='world'
                )

                pos_impedance = max(300.0, min(600.0, 600.0 - distance * 1000))
                
                self.fa.goto_pose(
                    target_transform,
                    duration=float(duration),
                    use_impedance=True,
                    cartesian_impedances=[pos_impedance, pos_impedance, pos_impedance, 50.0, 50.0, 50.0]
                )
            
            rospy.loginfo(f"Reached target position")
            
        except Exception as e:
            rospy.logerr(f"Error executing trajectory: {str(e)}")
        
        self.executing_trajectory = False
    
    def run(self):
        while not rospy.is_shutdown():
            if self.new_target_received and not self.executing_trajectory:
                self.execute_trajectory()
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