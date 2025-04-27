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
        translation=np.array([0.25, 0, 0.65]),
        rotation=np.array([[-0.00639471, -0.05183741,  0.99863506],
                            [ 0.69791764,  0.71496987,  0.04158193],
                            [-0.71614948,  0.69723093, 0.03160622]]),
        from_frame="franka_tool",
        to_frame="world"
    )


BOX_CORNER_MIN = np.array([0.4, -0.3, 0.2])
BOX_CORNER_MAX = np.array([0.66, 0.3, 0.8])

# World frame, offset from actual hand detection
X_OFFSET = -0.15
Y_OFFSET = +0.05
Z_OFFSET = +0.05
BUMP_DIST = 0.25
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

        self.disp = 0.02
        self.init_time = rospy.Time.now().to_time() # retrieve current time
        self.id = 0 # further used when moving to catch point

        self.last_validpose_time = time.time()
        self.last_invalidpose_time = time.time()
        self.state = 1
        self.bump_complete = False 
        self.goal_msg = None

        current = self.fa.get_pose()
        # print("Initial pose: ", current.translation) 
        self.fa.goto_pose(current, duration=self.dt, dynamic=True, buffer_time=500, block=False)
        
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop)

        # rospy.loginfo("Dynamic params setcurr_poseup")

    def set_goal(self, goal_msg):
        # self.goal_msg.x -= 0.3 #keep this dist from actual detection

        if not self._check_valid_pose(goal_msg):
            print("Invalid goal pose")
            self.last_invalidpose_time = time.time()
            self.goal_msg = None
            return
        
        # print("Got goal pose: ", goal_msg.x, goal_msg.y, goal_msg.z)
        self.last_validpose_time = time.time()
    
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
    
    def loop(self, _timerEvent):
        # print("Current state: ", self.state)
        # time.sleep(self.dt)
        if self.state == 0:
            # looking for fist, NOT tracking
            if self.goal_msg is not None:
                print("Got goal pose: ", self.goal_msg.x, self.goal_msg.y, self.goal_msg.z)
                self.state = 1
        elif self.state == 1:
            # tracking fist
            self.move()
            if self.goal_msg is None:
                print("Lost goal pose")
                self.state = 0
                return
            dist = np.linalg.norm(np.array([self.goal_msg.x + X_OFFSET, self.goal_msg.y + Y_OFFSET, self.goal_msg.z + Z_OFFSET]) - 
              self.fa.get_pose().translation)
            print("Distance to goal: ", dist)
            if dist < 0.05 and (time.time() - self.last_invalidpose_time > 1.0):
                print("Reached goal pose")
                self.state = 2
        elif self.state == 2:
            # reached goal pose, going in for bump
            self.bump()
            if self.bump_complete:
                # print("Bump complete")
                self.bump_complete = False
                self._dynamic_setup()
                self.state = 0          # TODO change this to 3 when 3 does something meaningful
        elif self.state == 3:
            # bump detected, going back to home pose
            print("Bump detected, going back to home pose")
            
        else:
            print("Unknown state, how did you get here?")
            self.state = 0

    def move(self):
        current_pose = self.fa.get_pose()
        current_pose.rotation = START_POSE.rotation

        if self.goal_msg is None:
            goal_pose = current_pose
        else:
            goal_pose = RigidTransform(
                translation=np.array([
                    self.goal_msg.x + X_OFFSET,
                    self.goal_msg.y + Y_OFFSET,
                    self.goal_msg.z + Z_OFFSET,
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
        # print("Published message")
        self.id += 1

    def bump(self):
        # print("Bumping")
        self.fa.stop_skill()
        current_pose = self.fa.get_pose()
        bump_pose = current_pose.copy()
        bump_pose.translation[0] += BUMP_DIST
        self.fa.goto_pose(bump_pose, duration=4.0, use_impedance=True,
            cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0], block=True)
        self.fa.goto_pose(START_POSE, duration=3.0, block=True)
        sensed_ft = self.fa.get_ee_force_torque()
        print("Sensed force: ", sensed_ft)
        if np.linalg.norm(sensed_ft) > 2.0:
            print("Bump detected")
            self.goal_msg = None
        else:
            print("No bump detected")
        self.bump_complete = True


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