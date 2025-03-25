#!/usr/bin/env python3
import sys
sys.path.append("/home/ros_ws")
from frankapy import FrankaArm
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose
from autolab_core import RigidTransform

class FistTracker(object):
    def __init__(self):
        self.fa = FrankaArm(init_node = False)

        self.latest_target = None
        self.is_executing = False

        self.target_sub = rospy.Subscriber('/target_pos', Point, self.update_target, queue_size=1)

        self.tracking_timer = rospy.Timer(rospy.Duration(0.1), self.track)

    def update_target(self, msg):
        self.latest_target = msg
        rospy.loginfo(f"Received target position: x={msg.x}, y={msg.y}, z={msg.z}")

    def track(self, msg):
        # convert target ee point to joint positions src/devel_packages/manipulation/src/moveit_class.py
        # plan and execute path to target 

        if self.is_executing or not self.latest_target:
            return

        target = self.latest_target
        
        try:
            self.is_executing = True
            current_pose = self.fa.get_pose()

            target_translation = np.array([target.x, target.y, target.z])
            target_rotation = np.array([0.01039112, 0.80840715, 0.00958253, 0.58844988])
        
            # Create a pose goal using the received point
            pose_goal = RigidTransform(
                translation=target_translation,
                rotation=RigidTransform.rotation_from_quaternion(target_rotation),
                from_frame='franka_tool',
                to_frame='world'
            )
        
            #Distance to target
            distance_to_target = np.linalg.norm(current_pose.translation - target_translation)
            duration = min(max(distance_to_target / 2.0, 0.2), 1.0)

            # Execute
            self.fa.goto_pose(
                pose_goal,
                duration=duration,
                dynamic=True,
                buffer_time=0.5
            )
            rospy.loginfo("Execution completed")
            self.is_executing = False

        except Exception as e:
            rospy.logerr(f"Error in tracking: {e}")

    def shutdown(self):
        self.tracking_timer.shutdown()
        rospy.loginfo("Gracefully shutting down")

if __name__ == '__main__':
    rospy.init_node('motion_tracking_node', anonymous=False)

    ft = FistTracker()
    rospy.on_shutdown(ft.shutdown)

    rospy.spin()