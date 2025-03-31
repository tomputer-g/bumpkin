#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import threading
import numpy as np

import tf2_ros
import tf_conversions
from geometry_msgs.msg import Point, Pose, TransformStamped
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import os

class AbstractPerceptionModel:
    def __init__(self):
        pass

    def predict(self, frame):
        pass

# Uses Google's MediaPipe Hand Landmarker model to detect hand landmarks
class MediapipeWrapper(AbstractPerceptionModel):
    def __init__(self, NUM_MAX_HANDS = 2):
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

    # Predicts centroids (by percentage of width/height) of hands in the frame
    def predict(self, frame):
        det_results_list = self.detector.detect(frame)
        det_centroids = []
        for i, det_results in enumerate(det_results_list.hand_landmarks):
            det_centroids.append(self._get_hand_centroid(det_results))
        return det_centroids

def display_thread(viewer):
    while not rospy.is_shutdown():
        if viewer.combined_img is not None:
            view = viewer.combined_img.copy()
            # print(view.dtype)
            
            # for idx in range(len(viewer.centroid_markers)):
            #     x, y = viewer.centroid_markers[idx][0], viewer.centroid_markers[idx][1]
            #     cv2.circle(view, (x, y), 5, (0, 255, 0), 2)
            #     cv2.putText(viewer.combined_img, f'Depth: {viewer.centroid_depths[idx]}', (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow('Combined Image', view)
            cv2.waitKey(1)
        rospy.sleep(1./15)


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
        self.model = MediapipeWrapper()

        display_thread_instance = threading.Thread(target=display_thread, args=(self,))
        display_thread_instance.start()

        self.bridge = CvBridge()
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.color_sub], queue_size=8, slop=0.01)
        self.ts.registerCallback(self.callback)
        # Get camera transform
        tfBuffer = tf2_ros.Buffer() #TODO do this in callback
        tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform("panda_link0", "camera_depth_optical_frame", rospy.Time(), rospy.Duration.from_sec(0.5)).transform
        pose = Pose(position=Point(x=trans.translation.x, y=trans.translation.y, z=trans.translation.z), orientation=trans.rotation)
        self.cam_to_world = tf_conversions.toMatrix(tf_conversions.fromMsg(pose))
        # print("Cam to World", self.cam_to_world)


    def _deproject_pixel_to_point_mm(self, x_cam, y_cam, depth):
        cam_frame_xy = np.array([x_cam, y_cam, 1])
        K_inv = np.linalg.inv(self.intrinsic)

        norm = K_inv @ cam_frame_xy

        x_mm = norm[0] * depth#y / 1000 - 0.032
        y_mm = norm[1] * depth
        z_mm = depth

        return (x_mm, y_mm, z_mm)

    def callback(self, depth_msg, color_msg):
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
            
            centroids = self.model.predict(_frame)
            self.centroid_markers.clear()
            self.centroid_depths.clear()
            for centroid in centroids:
                x, y = int(centroid[0] * self.depth_image.shape[1])-1, int(centroid[1] * self.depth_image.shape[0])-1
                # print("Centroid: ({},{})".format(centroid[0], centroid[1]))
                # print("Hand centroid at: ({}, {})".format(x, y))
                depth_value = self.depth_image[y, x] #test this
                # print("Depth at centroid: {}".format(depth_value))
                x_cam_mm, y_cam_mm, z_cam_mm = self._deproject_pixel_to_point_mm(x_cam=x, y_cam=y, depth=depth_value)
                print("Pose in camera frame: ({:.3f}mm, {:.3f}mm, {:.3f}mm)".format(x_cam_mm, y_cam_mm, z_cam_mm))

                x_world_m, y_world_m, z_world_m, _ = np.matmul(self.cam_to_world, np.array([x_cam_mm / 1000, y_cam_mm / 1000 , z_cam_mm / 1000, 1]))
                print("Pose in world frame: ({:.3f}m, {:.3f}m, {:.3f}m)".format(x_world_m, y_world_m, z_world_m))

                # publish_point_tf(x_world_m, y_world_m, z_world_m)

                self.centroid_markers.append([int(color_image_resized.shape[1] * centroid[0]), int(color_image_resized.shape[0] * centroid[1])])
                self.centroid_depths.append(depth_value)

      
            for idx in range(len(self.centroid_markers)):
                x, y = self.centroid_markers[idx][0], self.centroid_markers[idx][1]
                cv2.circle(self.combined_img, (x, y), 5, (0, 255, 0), 2)
                cv2.putText(self.combined_img, f'Depth: {self.centroid_depths[idx]}', (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('realsense_viewer', anonymous=True)
    perception = BumpkinPerception()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()