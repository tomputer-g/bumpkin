import pyrealsense2 as rs
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
        return (x_avg, y_avg)

    # Predicts centroids (by percentage of width/height) of hands in the frame
    def predict(self, frame):
        det_results_list = self.detector.detect(frame)
        det_centroids = []
        for i, det_results in enumerate(det_results_list.hand_landmarks):
            det_centroids.append(self._get_hand_centroid(det_results))
        return det_centroids


def main():

    # Configure depth and color streams
    WIDTH, HEIGHT = 640,480

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue
            print("---------------------------------------")
            # print("asdf")
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))
            
            rgb_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=color_image)
            model = MediapipeWrapper()
            centroids = model.predict(rgb_frame)
            for centroid in centroids:
                x, y = int(centroid[0] * WIDTH), int(centroid[1] * HEIGHT)
                print("Hand centroid at: ({}, {})".format(x, y))
                depth_value = depth_image[y, x] #test this
                print("Depth at centroid: {}".format(depth_value))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)
            cv2.imshow('RealSense', images)
            cv2.waitKey(1)

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()
        
if __name__ == "__main__":
    main()