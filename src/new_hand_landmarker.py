#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import message_filters
import threading
import cv2
import numpy as np

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

class BumpkinPerception:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.color_sub], queue_size=8, slop=0.01)
        self.ts.registerCallback(self.callback)
        self.combined_img = None
        self.centroid_markers = []
        self.centroid_depths = []

        self.model = MediapipeWrapper()

        display_thread_instance = threading.Thread(target=display_thread, args=(self,))
        display_thread_instance.start()


    def callback(self, depth_msg, color_msg):
        rospy.loginfo("-----------------Callback loop-----------------")
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            print(self.depth_image.shape) #480,848 but 848 is width
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
                print("Centroid: ({},{})".format(centroid[0], centroid[1]))
                print("Hand centroid at: ({}, {})".format(x, y))
                depth_value = self.depth_image[y, x] #test this
                print("Depth at centroid: {}".format(depth_value))

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