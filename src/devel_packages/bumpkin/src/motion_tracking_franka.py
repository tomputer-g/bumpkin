#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import numpy as np
from frankapy import FrankaArm, SensorDataMessageType
from autolab_core import RigidTransform
from scipy.spatial.transform import Rotation
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

class FrankaTrajectoryExecutor:
    def __init__(self):

        self.rate = rospy.Rate(50)  # 50Hz control loop
        self.id = 0
        
        # Initialize
        rospy.loginfo("Initializing FrankaArm...")
        self.fa = FrankaArm(init_node=False)

        self.current_target = None
        self.new_target_received = False
        self.executing_trajectory = False
        self.current_pose = self.fa.get_pose()

        # EE orientation
        self.default_orientation = np.array([[0.42156439, -0.011119, 0.90672841],
                                    [-0.01462131,-0.99986855,-0.00546339],
                                    [ 0.90666997,-0.01095438,-0.42167967]])
        
        self.target_sub = rospy.Subscriber('/target_pos', Point, self.target_callback)

        # self.fa_sensor_pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)

        # current = self.fa.get_pose()
        # self.fa.goto_pose(current, duration=5, dynamic=True, buffer_time=10, cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0])
        # self.init_time = rospy.Time.now().to_time()
        
        # Safety limit distance
        self.max_position_change = 0.10  # 20cm
        
        rospy.loginfo("Franka ready.")      
    
    def target_callback(self, msg):
        # Callback for receiving new target positions

        # limits
        # Tra: [ 0.37531122 -0.31646363  0.6887249 ] top left
        # Tra: [ 0.44450838 -0.30382788  0.18063057] bot left 
        # Tra: [0.30388335 0.27008619 0.75117503] top right
        # Tra: [0.32392089 0.35606499 0.24398415] bot right

        # Tra: [ 0.60660365 -0.24438622  0.56997307] outwards
        # Tra: [ 0.28004865 -0.2835988   0.59593051] inwards
        # [z, x, y]
        # HOME_POSE = RigidTransform(rotation=np.array([
        #     [1, 0, 0],
        #     [0, -1, 0],
        #     [0, 0, -1],
        # ]), translation=np.array([0.3069, 0, 0.4867]),
        # from_frame='franka_tool', to_frame='world')
        if np.abs(msg.y) > 0.3 or msg.z < 0.2 or msg.z > 0.7 or msg.x < 0.25 or msg.x > 0.55: 
            rospy.logwarn("Target outside of workspace bounds, ignoring")
            return
 
        if self.executing_trajectory:
            rospy.logwarn("Received new target while executing trajectory, skip")
            return
        
        target_position = np.array([msg.x, msg.y, msg.z])

        # Safety check - limit maximum position change
        current_position = self.fa.get_pose().translation
        distance_to_target = np.linalg.norm(target_position - current_position)
        
        if distance_to_target > self.max_position_change:
            rospy.logwarn(f"Target position too far ({distance_to_target:.2f}m). Limiting movement to {self.max_position_change}m.")
            direction = (target_position - current_position) / distance_to_target
            target_position = current_position + direction * self.max_position_change
        
        if self.current_target is not None:

            distance = np.linalg.norm(target_position - self.current_target)
            
            # If the target hasn't changed significantly, ignore
            if distance < 0.05:  # 5cm threshold
                return
            
            # elif self.executing_trajectory:
            #     rospy.logwarn(f"New target received. Replanning to new target")
            #     self.fa.stop_skill()
        
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
                    cartesian_impedances=[pos_impedance, pos_impedance, pos_impedance, 50.0, 50.0, 50.0],
                    block=False
                )

                # timestamp = rospy.Time.now().to_time() - self.init_time

                # traj_gen_msg = PosePositionSensorMessage(
                #     id=self.id,
                #     timestamp=timestamp,
                #     position=target_transform.translation,
                #     #TODO: Check if the START_POSE is the same name being maintained
                #     quaternion=target_transform.quaternion,
                # )
                # ros_msg = make_sensor_group_msg(
                #     trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                #         traj_gen_msg, SensorDataMessageType.POSE_POSITION
                #     )
                # )
                # self.fa_sensor_pub.publish(ros_msg)
                # self.id += 1
            
            rospy.loginfo(f"Reached target position")
            
        except Exception as e:
            rospy.logerr(f"Error executing trajectory: {str(e)}")
        
        self.fa.stop_skill()
        self.executing_trajectory = False
    
    def run(self):
        while not rospy.is_shutdown():
            if self.new_target_received:
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