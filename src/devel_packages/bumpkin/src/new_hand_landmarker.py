#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import threading
import numpy as np
from motion_tracking_franka import FrankaTrajectoryExecutor

import tf2_ros
import tf_conversions
from geometry_msgs.msg import Point, Pose, TransformStamped
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import os

from norfair import Detection, Tracker
import time

CHANNEL = "/target_pos"
MAX_DIST_FROM_CAMERA = 0.5 #meters, beyond this distance from camera detections are ignored
NUM_MAX_HANDS = 1

FPS = 30 #Should be same as realsense.launch definitions. Not to exceed 30 (mediapipe takes max 30ms)

### Norfair
DIST_THRESHOLD_BETWEEN_FRAMES = 120
HIT_COUNTER_MAX = 10

# Uses Google's MediaPipe Hand Landmarker model to detect hand landmarks
class MediapipeWrapper():
    def __init__(self):
        if not os.path.exists('hand_landmarker.task'):
            print("The model file 'hand_landmarker.task' does not exist. Attempting to fetch...")
            os.system('wget -q https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task')
        base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
        options = vision.HandLandmarkerOptions(base_options=base_options,
                                            num_hands=NUM_MAX_HANDS)
        self.detector = vision.HandLandmarker.create_from_options(options)

    # Helper function to get centroid of hand landmarks
    def _get_hand_centroid(self, hand_landmarks):
        x_avg, y_avg = 0, 0
        for landmark in hand_landmarks:
            x_avg += landmark.x
            y_avg += landmark.y
        x_avg /= len(hand_landmarks)
        y_avg /= len(hand_landmarks)
        #Prevent OOB predictions, which mediapipe does
        x_avg = min(1, max(0, x_avg))
        y_avg = min(1, max(0, y_avg))
        return (x_avg, y_avg)
    
    def _get_middle_finger_feats(self, hand_landmarks):
        # see https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker
        mid_finger_mcp = hand_landmarks[9]
        mid_finger_pip = hand_landmarks[10]
        return [(mid_finger_mcp.x, mid_finger_mcp.y, mid_finger_mcp.z), (mid_finger_pip.x, mid_finger_pip.y, mid_finger_pip.z)]
    
    def _get_hand_bbox(self, hand_landmarks):
        x_min = min([landmark.x for landmark in hand_landmarks])
        x_max = max([landmark.x for landmark in hand_landmarks])
        y_min = min([landmark.y for landmark in hand_landmarks])
        y_max = max([landmark.y for landmark in hand_landmarks])
        # Prevent OOB predictions, which mediapipe does
        x_min = min(1, max(0, x_min))
        x_max = min(1, max(0, x_max))
        y_min = min(1, max(0, y_min))
        y_max = min(1, max(0, y_max))
        return (x_min, y_min, x_max, y_max)

    # Predicts centroids (by percentage of width/height) of hands in the frame
    def predict(self, frame):
        det_results_list = self.detector.detect(frame)
        det_centroids = []
        det_bboxes = []
        det_middle_fings = []
        for i, det_results in enumerate(det_results_list.hand_landmarks):
            det_centroids.append(self._get_hand_centroid(det_results))
            det_bboxes.append(self._get_hand_bbox(det_results))
            det_middle_fings.append(self._get_middle_finger_feats(det_results))
        return det_centroids, det_bboxes, det_middle_fings

display_img = None
def display_thread(perceptionThread):
    while not rospy.is_shutdown():
        if display_img is not None:
            cv2.imshow('Combined Image', display_img)
            cv2.waitKey(1)
        rospy.sleep(1./FPS)


def publish_point_tf(x, y, z):
    br = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "target_point"
    
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    
    # No rotation (identity quaternion)
    t.transform.rotation.w = 1.0
    
    br.sendTransform(t)
    rospy.spin()

class BumpkinPerception:
    def __init__(self):

        self.combined_img = None
        self.centroid_markers = []
        self.centroid_depths = []

        camera_info = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
        self.intrinsic = np.array(camera_info.K).reshape((3, 3))
        # print(self.intrinsic)
        self.model = MediapipeWrapper()
        self.target = None

        display_thread_instance = threading.Thread(target=display_thread, args=(self,))
        display_thread_instance.start()

        # target_pub_thread_instance = threading.Thread(target=target_publisher, args=(self,))
        # target_pub_thread_instance.start()
        self.target_pub = rospy.Publisher('/target_pos', Point, queue_size=10)

        self.bridge = CvBridge()
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.color_sub], queue_size=15, slop=0.01)
        self.ts.registerCallback(self.callback)

        self.real_world_mid_finger_esti = 0.098
        # Get camera transform
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        
        # print("Cam to World", self.cam_to_world)
        self.tracker = Tracker(distance_function='euclidean', distance_threshold=DIST_THRESHOLD_BETWEEN_FRAMES, hit_counter_max=HIT_COUNTER_MAX)

    def _get_cam_transform(self):
        trans = self.tfBuffer.lookup_transform("panda_link0", "camera_depth_optical_frame", rospy.Time(), rospy.Duration.from_sec(0.5)).transform
        pose = Pose(position=Point(x=trans.translation.x, y=trans.translation.y, z=trans.translation.z), orientation=trans.rotation)
        return tf_conversions.toMatrix(tf_conversions.fromMsg(pose))

    def _deproject_pixel_to_point_mm(self, x_cam, y_cam, depth):
        cam_frame_xy = np.array([x_cam, y_cam, 1])
        K_inv = np.linalg.inv(self.intrinsic)

        norm = K_inv @ cam_frame_xy

        x_mm = norm[0] * depth#y / 1000 - 0.032
        y_mm = norm[1] * depth
        z_mm = depth

        return (x_mm, y_mm, z_mm)
    
    def _get_est_depth_from_lengths(self, length_cam, length_real):
        # fx and fy are the same
        f = self.intrinsic[0][0]
        depth = f * length_real / length_cam
        return depth

    def callback(self, depth_msg, color_msg):
        global display_img
        # rospy.loginfo("-----------------Callback loop-----------------")
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            # print(self.depth_image.shape) #480,848 but 848 is width
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            self.color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
            color_image_resized = cv2.resize(color_image, (640, 480))
            depth_image_resized = cv2.resize(depth_colormap, (640, 480))
            combined_image = cv2.hconcat([color_image_resized, depth_image_resized])
            self.combined_img = combined_image

            _frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=self.color_image)
            centroids, bboxes, middle_finger_stats = self.model.predict(_frame)
            # print("\rCentroids: ", centroids)

            self.centroid_markers.clear()
            self.centroid_depths.clear()

            cam_to_world = self._get_cam_transform()

            norfair_detections = []
            for centroid_idx in range(len(centroids)):
                centroid = centroids[centroid_idx]
                bbox = bboxes[centroid_idx]
                x, y = int(centroid[0] * self.depth_image.shape[1])-1, int(centroid[1] * self.depth_image.shape[0])-1
                # Get depth values in a 21x21 window around the centroid
                depth_window = self.depth_image[max(0, y-10):min(self.depth_image.shape[0], y+11), 
                                                max(0, x-10):min(self.depth_image.shape[1], x+11)]
                median_depth_value = np.median(depth_window)
                min_depth_value = np.min(depth_window)
                depth_value = median_depth_value

                # print("median depth is", median_depth_value)
                mid_finger_stats = middle_finger_stats[centroid_idx][0]
                # print(mid_finger_stats)
                mid_finger_mcp_x, mid_finger_mcp_y, mid_finger_mcp_z =middle_finger_stats[centroid_idx][0]
                mid_finger_pip_x, mid_finger_pip_y, mid_finger_pip_z =middle_finger_stats[centroid_idx][1]
                
                # Estimate depth based on self.real_world_mid_finger_esti
                px_length = (mid_finger_mcp_x - mid_finger_pip_x)**2 + (mid_finger_mcp_y - mid_finger_pip_y)**2 + (mid_finger_mcp_z - mid_finger_pip_z)**2
                px_length = np.sqrt(px_length)
                esti_depth = self._get_est_depth_from_lengths(length_cam=px_length, length_real=self.real_world_mid_finger_esti)
                # print("Estimated depth is", esti_depth)
                # deproject
                mid_finger_mcp_realw_x, mid_finger_mcp_realw_y, mid_finger_mcp_realw_z = self._deproject_pixel_to_point_mm(mid_finger_mcp_x, mid_finger_mcp_y, depth_value)
                mid_finger_pip_realw_x, mid_finger_pip_realw_y, mid_finger_pip_realw_z = self._deproject_pixel_to_point_mm(mid_finger_pip_x, mid_finger_pip_y, depth_value)
                # print("realw dist", np.sqrt( (mid_finger_mcp_realw_x-mid_finger_pip_realw_x)**2 + (mid_finger_mcp_realw_y-mid_finger_pip_realw_y)**2 + (mid_finger_mcp_realw_z-mid_finger_pip_realw_z)**2))
                
                # print(mid_finger_stats)

                # Middle finger 

                """
                Using the picture of a fist on a pipe here are the values (actual depth, inch | dist in percentage img)
                4 in | 0.3826
                6 in | 0.2708
                8 in | 0.2544
                10 in | 0.2169
                12 in (with depth output 303mm) | 0.1777
                14 in (with depth output 339mm) | 0.1255
                """
                # if min_depth_value <= 0:
                #     print("Ignoring object at ({}, {}) due to invalid minimum depth in window".format(x, y))
                #     cv2.rectangle(self.combined_img, (int(bbox[0] * color_image_resized.shape[1]), int(bbox[1] * color_image_resized.shape[0])), (int(bbox[2] * color_image_resized.shape[1]), int(bbox[3] * color_image_resized.shape[0])), (0, 0, 255), 2)
                #     continue

                # depth_value = self.depth_image[y, x]


                # if depth_value == 0 and esti_depth > 0:
                #     # print("Using estimated depth")
                #     depth_value = esti_depth


                if depth_value == 0:
                    # print("Ignoring object at ({}, {}) with depth 0".format(x, y))
                    cv2.rectangle(self.combined_img, (int(bbox[0] * color_image_resized.shape[1]), int(bbox[1] * color_image_resized.shape[0])), (int(bbox[2] * color_image_resized.shape[1]), int(bbox[3] * color_image_resized.shape[0])), (0, 0, 255), 2)
                    continue
                if depth_value / 1000.0 > MAX_DIST_FROM_CAMERA:
                    # print("Ignoring object at ({}, {}) with depth {:.3f}m beyond max distance from camera".format(x, y, depth_value / 1000.0))
                    cv2.rectangle(self.combined_img, (int(bbox[0] * color_image_resized.shape[1]), int(bbox[1] * color_image_resized.shape[0])), (int(bbox[2] * color_image_resized.shape[1]), int(bbox[3] * color_image_resized.shape[0])), (0, 0, 255), 2)
                    continue

                cv2.rectangle(self.combined_img, (int(bbox[0] * color_image_resized.shape[1]), int(bbox[1] * color_image_resized.shape[0])), (int(bbox[2] * color_image_resized.shape[1]), int(bbox[3] * color_image_resized.shape[0])), (0, 255, 0), 2)

                # print("Depth at centroid: {}".format(depth_value))
                x_cam_mm, y_cam_mm, z_cam_mm = self._deproject_pixel_to_point_mm(x_cam=x, y_cam=y, depth=depth_value)
                # print("Pose in camera frame: ({:.3f}mm, {:.3f}mm, {:.3f}mm)".format(x_cam_mm, y_cam_mm, z_cam_mm))

                

                x_world_m, y_world_m, z_world_m, _ = np.matmul(cam_to_world, np.array([x_cam_mm / 1000, y_cam_mm / 1000 , z_cam_mm / 1000, 1]))
                print("Pose in world frame: ({:.3f}m, {:.3f}m, {:.3f}m)".format(x_world_m, y_world_m, z_world_m))
                self.target = Point()
                            
                self.target.x = x_world_m
                self.target.y = y_world_m
                self.target.z = z_world_m
                # publish_point_tf(x_world_m, y_world_m, z_world_m)
                self.target_pub.publish(self.target)

                self.centroid_markers.append([int(color_image_resized.shape[1] * centroid[0]), int(color_image_resized.shape[0] * centroid[1])])
                self.centroid_depths.append(depth_value)

                # Create a Norfair detection
                norfair_detection = Detection(points=np.array([[int(color_image_resized.shape[1] * centroid[0]), int(color_image_resized.shape[0] * centroid[1])]]))
                norfair_detections.append(norfair_detection)
            # Update the tracker with the detections
            self.tracker.update(detections=norfair_detections)
            # Draw the tracked objects on the image
            for tracked_object in self.tracker.tracked_objects:
                for point in tracked_object.estimate:
                    x, y = int(point[0]), int(point[1])
                    cv2.circle(self.combined_img, (x, y), 5, (255, 0, 0), 2)
            display_img = self.combined_img.copy()
            for idx in range(len(self.centroid_markers)):
                x, y = self.centroid_markers[idx][0], self.centroid_markers[idx][1]
                cv2.circle(self.combined_img, (x, y), 5, (0, 255, 0), 2)
                cv2.putText(self.combined_img, f'Depth: {self.centroid_depths[idx]}', (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            
            
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('realsense_viewer', anonymous=True)
    perception = BumpkinPerception()

    # franka_executor = FrankaTrajectoryExecutor()

    # rospy.on_shutdown(franka_executor.shutdown)

    # thread = threading.Thread(target=franka_executor.run)
    # thread.daemon = True
    # thread.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()