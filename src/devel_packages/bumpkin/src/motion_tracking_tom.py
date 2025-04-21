#!/usr/bin/env python3

import time
import numpy as np
import rospy
from geometry_msgs.msg import Point

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

START_POSE = RigidTransform(
        translation=np.array([0.25, 0, 0.55]),
        rotation=np.array([[-0.00639471, -0.05183741,  0.99863506],
                            [ 0.69791764,  0.71496987,  0.04158193],
                            [-0.71614948,  0.69723093, 0.03160622]]),
        from_frame="franka_tool",
        to_frame="world"
    )


BOX_CORNER_MIN = np.array([0.46, -0.3, 0.2])
BOX_CORNER_MAX = np.array([0.66, 0.3, 0.7])
class BumpkinPlanner:
    

    def _dynamic_setup(self):
        # retrieve goal_pose position:
        self.sub = rospy.Subscriber(
            "/target_pos",
            Point,
            self.set_goal
        )

        self.pub = rospy.Publisher(
            FC.DEFAULT_SENSOR_PUBLISHER_TOPIC,
            SensorDataGroup,
            queue_size=1000
        )
        # print(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC)

        self.dt = 0.01
        # self.rate = rospy.Rate(1 / self.dt)

        self.disp = 0.05
        self.init_time = rospy.Time.now().to_time() # retrieve current time
        self.id = 0 # further used when moving to catch point

        self.last_validpose_time = time.time()
        self.goal_msg = None

        current = self.fa.get_pose()
        # print("Initial pose: ", current.translation) 
        self.fa.goto_pose(current, duration=self.dt, dynamic=True, buffer_time=500, block=False)
        
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.move)

        # rospy.loginfo("Dynamic params setcurr_poseup")

    def set_goal(self, goal_msg):
        # self.goal_msg.x -= 0.3 #keep this dist from actual detection

        if not self._check_valid_pose(goal_msg):
            print("Invalid goal pose")
            return
    
        self.goal_msg = goal_msg

    def _check_valid_pose(self, goal_msg):
        translation = [goal_msg.x, goal_msg.y, goal_msg.z]

        if np.any(np.isnan(translation)):
            print("Pose has NaN")
            return False

        if (translation < BOX_CORNER_MIN).any() or (translation > BOX_CORNER_MAX).any():
            print("Pose out of bounds")
            return False

        return True
    
    def move(self, _timerEvent):
        current_pose = self.fa.get_pose()

        if self.goal_msg is None:
            goal_pose = current_pose
        else:
            goal_pose = RigidTransform(
                translation=np.array([
                    self.goal_msg.x,
                    self.goal_msg.y,
                    self.goal_msg.z,
                ]),
                rotation=current_pose.rotation,
                from_frame=current_pose.from_frame,
                to_frame=current_pose.to_frame
            )

        # Calculate delta
        delta = goal_pose.translation - current_pose.translation
        delta_norm = np.linalg.norm(delta)

        # Limit movement
        if delta_norm > self.disp:
            delta = (delta / delta_norm) * self.disp

        waypoint = current_pose.copy()
        waypoint.translation += delta
        timestamp = rospy.Time.now().to_time() - self.init_time

        # print("going to waypoint: ", waypoint.translation)

        traj_gen_msg = PosePositionSensorMessage(
            id=self.id,
            timestamp=timestamp,
            position=waypoint.translation,
            #TODO: Check if the START_POSE is the same name being maintained
            quaternion=current_pose.quaternion,
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_msg, SensorDataMessageType.POSE_POSITION
            )
        )
        self.pub.publish(ros_msg)
        self.id += 1

    def __init__(self):
        self.fa = FrankaArm(with_gripper=False)
        self.fa.goto_pose(START_POSE, block=True)
        rospy.loginfo("at home pose")
        self._dynamic_setup()
    
    def shutdown(self):
        self.fa.stop_skill() # stops the current skill 


     
if __name__ == "__main__":
    planner = BumpkinPlanner()
    while not rospy.is_shutdown():
        rospy.spin()
    planner.shutdown() # stops the current skill 