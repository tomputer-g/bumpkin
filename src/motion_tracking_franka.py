#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import numpy as np
from frankapy import FrankaArm
from autolab_core import RigidTransform
from scipy.spatial.transform import Rotation

class FrankaTrajectoryExecutor:
    def __init__(self):

        self.rate = rospy.Rate(100)  # 1000Hz control loop
        
        # Initialize
        rospy.loginfo("Initializing FrankaArm...")
        self.fa = FrankaArm(init_node=False)

        self.current_target = None
        self.new_target_received = False
        self.executing_trajectory = False
        self.current_pose = self.fa.get_pose()

        # EE orientation
        # quaternion = [0.01039112, 0.80840715, 0.00958253, 0.58844988]
        # orientation = Rotation.from_quat(quaternion)
        # self.default_orientation = orientation.as_matrix()
        self.default_orientation = np.array([[0.42156439, -0.011119, 0.90672841],
                                    [-0.01462131,-0.99986855,-0.00546339],
                                    [ 0.90666997,-0.01095438,-0.42167967]])
        
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
            target_transform = RigidTransform(
                rotation=self.default_orientation,
                translation=self.current_target,
                from_frame='franka_tool',
                to_frame='world'
            )
            
            rospy.loginfo(f"Moving to target: {self.current_target}")
            
            self.fa.goto_pose(
                target_transform,
                duration=0.2, #sec
                use_impedance=True,
                cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
            )
            
            rospy.sleep(0.1)
            
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
        executor = FrankaTrajectoryExecutor()
        rospy.on_shutdown(executor.shutdown)
        executor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received, shutting down")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {str(e)}")