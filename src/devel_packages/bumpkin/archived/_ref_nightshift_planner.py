#!/usr/bin/env python3

# This file is reference from https://github.com/shrudh-i/16662-ThrowCatchSort/blob/main/src/nightshift-planner/src/nightshift_planner.py
import time
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage

VERBOSE = True

from franka_interface_msgs.msg import SensorDataGroup

class PlannerNode:

    #TODO: verify the tool pose for our end-effector
    NIGHTSHIFT_TOOL_POSE = RigidTransform(
            translation=np.array([0.106, -0.106, -0.01]),
            rotation=np.array([[1, 0, 0],
                                [0, 1, 0],
                                [0,  0, 1]]),
            from_frame="franka_tool",
            to_frame="franka_tool_base"
        )

    #TODO: verify if this is a good pose from which the arm can start moving from
    START_POSE = RigidTransform(
            translation=np.array([0.56, 0, 0.4]),
            rotation=np.array([[7.07185286e-01, -7.07028188e-01, -3.36139057e-04],
                                [-7.07028245e-01, -7.07185062e-01, -5.90835436e-04],
                                [1.80024788e-04,  6.55489934e-04, -9.99999769e-01]]),
            from_frame="franka_tool",
            to_frame="world"
        )

    #TODO: verify if this bounding box is what we want to go with
    BOX_CORNER_MIN = np.array([0.46, -0.3, 0.2])
    BOX_CORNER_MAX = np.array([0.66, 0.3, 0.7])


    @classmethod
    def __init__(self):
        self.fa = FrankaArm() 
        self.fa.set_tool_delta_pose(self.NIGHTSHIFT_TOOL_POSE)
        self.reset()
    
    @classmethod
    def reset(self):
        '''
        Reset the Franka Arm
        '''

        if VERBOSE: 
            rospy.loginfo("Resetting the Franka Arm")

        self.fa.reset_joints()
        self.fa.stop_skill() # stops the current skill 
        self.fa.wait_for_skill() # waits for the current skill to be stopped

        self.dynamic_setup()

        if VERBOSE:
            rospy.loginfo("Reset Complete")

    
    @classmethod
    def dynamic_setup(self):
        '''
        Setting up dynamic params before catching
        '''

        if VERBOSE:
            rospy.loginfo("Setting up dynamic params")

        # init the goal_pose 
        # self.goal_msg = None

        # retrieve goal_pose position:
        self.sub = rospy.Subscriber(
            #TODO: Confirm on the topic name
            "",
            PointStamped,
            self.set_goal()
        )

        self.pub = rospy.Publisher(
            FC.DEFAULT_SENSOR_PUBLISHER_TOPIC,
            SensorDataGroup,
            queue_size=1000
        )
        print(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC)

        #TODO: Check if this is required
        #TODO: Confirm the speed at which we want the arm to move (duration)
        self.dt = 0.01
        # self.rate = rospy.Rate(1 / self.dt)

        self.disp = 0.2
        self.init_time = rospy.Time.now().to_time() # retrieve current time
        self.id = 0 # further used when moving to catch point

        self.last_validpose_time = time.time()

        self.timer = rospy.Timer(rospy.Duration(self.dt), self.moveToCatch)
        self.timeout = 5

        current = self.fa.get_pose()
        print("Initial pose: ", current.translation) 
        self.fa.goto_pose(current, duration=self.dt, dynamic=True, buffer_time=500, block=False)

        if VERBOSE:
            rospy.loginfo("Dynamic params setcurr_poseup")


    @classmethod
    def set_goal(self, goal_msg):
        '''
        Callback to store the subscribed goal_pose pose 
        '''
        self.goal_msg = goal_msg

    @classmethod
    def catch(self):
        '''
        Moving the Franka Arm to the catch point
        '''

        current = self.fa.get_pose()
        if np.any(np.isnan(current.translation)) or np.any(np.isnan(current.quaternion)):
            print("Current pose has Nan")

        if self.goal_msg is None:
            goal_pose = current
        else:
            goal_pose = RigidTransform(
                translation = np.array([
                    self.goal_msg.point.x,
                    self.goal_msg.point.y,
                    self.goal_msg.point.z,
                ]),# A good pose from where robot can start moving from
                from_frame=current.from_frame,
                to_frame=current.to_frame,
            )

        # store a copy of the current arm pose
        intermediate = current

        displacement = goal_pose.translation - current.translation
        displ_norm = np.norm(displacement)

        #TODO: verify if this heuristic holds
        if np.abs(displ_norm) > 1e-3:
            displacement = min(displ_norm, self.disp) * (displacement/displ_norm)
        intermediate.translation = current.translation + displacement
        timestamp = rospy.Time.now().to_time() - self.init_time

        if not self.is_valid_pose(intermediate):
            print("Invalid Pose:", intermediate.translation)
            interpol_pose = current
        else:
            self.last_validpose_time = time.time()

        if np.any(np.isnan(intermediate.translation)):
            intermediate = current
        #TODO: Check if the TOOL_DELTA_POSE is the same name being maintained
        intermediate = intermediate * self.TOOL_DELTA_POSE.inverse()

        traj_gen_msg = PosePositionSensorMessage(
            id=self.id,
            timestamp=timestamp,
            position=intermediate.translation,
            #TODO: Check if the START_POSE is the same name being maintained
            quaternion=self.START_POSE.quaternion,
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_msg, SensorDataMessageType.POSE_POSITION
            )
        )

        if np.any(np.isnan(intermediate.translation)):
            print("Nan in message")
        self.pub.publish(ros_msg)
        self.id += 1

        if (time.time() - self.last_validpose_time > self.timeout):
            self.is_resetting = True
            print('Terminating...')
            self.terminate_dynamic()
            print("Terminated")
            self.reset()
        
        ###################TEMP####################

        # print("Current error: ", np.linalg.norm(curr_pose.translation - goal_pose.translation))
        # print("Interpol pose: ", interpol_pose.translation)
        # print("Delta: ", delta)

    @classmethod
    def terminate_dynamic(self, pose):
        '''
        '''
        return np.all(pose.translation <= self.BOX_CORNER_MAX) and np.all(pose.translation >= self.BOX_CORNER_MIN)

    @classmethod
    def is_valid_pose(self):
        '''
        '''
        terminate_msg = ShouldTerminateSensorMessage(
        timestamp=rospy.Time.now().to_time() - self.init_time, should_terminate=True
        )
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                terminate_msg, SensorDataMessageType.SHOULD_TERMINATE
            )
        )
        self.pub.publish(ros_msg)
        self.fa.wait_for_skill()