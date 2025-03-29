#!/usr/bin/env python3

import yaml
import rospy
import rospkg
import imutils
import tf2_ros
import tf_conversions
import message_filters
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker

from std_msgs.msg import Float32

import cv2
import numpy as np
# from ultralytics import YOLO

VERBOSE = True
VISUALIZE = True

class TrackingNode():
	
    @classmethod
    def __init__(self) -> None:
        # Input: Self
		# Output: None

        rospy.init_node('track_ball', anonymous=True)
        self.verbose = VERBOSE
        self.visualize = VISUALIZE
        # Get camera info for 3D conversion
        if self.verbose: rospy.loginfo("Waiting for camera_info...")
        camera_info = rospy.wait_for_message('rgb/camera_info', CameraInfo)
        self.intrinsic = np.array(camera_info.K).reshape((3, 3))

        # Initialize publishers and subscribers
        image_sub = message_filters.Subscriber('rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/depth_to_rgb/image_raw', Image)
        self.landing_pos_pub = rospy.Publisher('landing_pos', Point)

        # NOTE: Temporary publishers
        self.x_pub = rospy.Publisher('x_pos', Float32, queue_size=10)
        self.y_pub = rospy.Publisher('y_pos', Float32, queue_size=10)
        self.z_pub = rospy.Publisher('z_pos', Float32, queue_size=10)

        if self.visualize:
            self.yolo_pub = rospy.Publisher('detections',Image,queue_size=10)
            self.ball_pub = rospy.Publisher("ball_marker", Marker, queue_size=10)
            self.landing_pub = rospy.Publisher("landing_marker", Marker, queue_size=10)
            
            # Define ball marker
            self.ball_marker = Marker()
            self.ball_marker.header.frame_id = "panda_link0"
            self.ball_marker.type = 2
            self.ball_marker.id = 0
            self.ball_marker.color.r = 1.0
            self.ball_marker.color.g = 0.65
            self.ball_marker.color.b = 0.0
            self.ball_marker.color.a = 1.0
            self.ball_marker.scale.x = 0.05
            self.ball_marker.scale.y = 0.05
            self.ball_marker.scale.z = 0.05
            self.ball_marker.pose.orientation.w = 1

            # Define landing marker
            self.landing_marker = Marker()
            self.landing_marker.header.frame_id = "panda_link0"
            self.landing_marker.type = 3
            self.landing_marker.id = 0
            self.landing_marker.color.r = 1.0
            self.landing_marker.color.g = 0.0
            self.landing_marker.color.b = 0.0
            self.landing_marker.color.a = 1.0
            self.landing_marker.scale.x = 0.05
            self.landing_marker.scale.y = 0.05
            self.landing_marker.scale.z = 0.01
            self.landing_marker.pose.orientation.w = 1

        # Create time synchronizer for image and depth
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=8, slop=0.01)
        ts.registerCallback(self.callback)

        self.cv_bridge = CvBridge()
        # self.pos_hist = [(np.zeros(3), 0) for i in range(self.position_history_length)]
        # self.pos_hist_filt = [(np.zeros(3), 0) for i in range(self.position_history_length)]
        # self.landing_pos_hist = [Point(x=np.inf, y=np.inf, z=np.inf) for i in range(self.position_history_length + 1)]
        # self.landing_pos = Point(x=np.inf, y=np.inf, z=np.inf)

        # Get camera transform
        tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform("panda_link0", "camera_base", rospy.Time(), rospy.Duration.from_sec(0.5)).transform
        pose = Pose(position=Point(x=trans.translation.x, y=trans.translation.y, z=trans.translation.z), orientation=trans.rotation)
        self.cam_to_world = tf_conversions.toMatrix(tf_conversions.fromMsg(pose))

        if self.verbose: rospy.loginfo("Init Done!")

    @classmethod
    def detect(self, img):
        pts = deque(maxlen=64)

        # greenLower = (29, 86, 6)
        # greenUpper = (64, 255, 255)

        # actually red
        greenLower = (117, 197, 103)
        greenUpper = (255, 255, 255)
        # img = imutils.resize(img, width=600)
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        green_mask = cv2.inRange(hsv, greenLower, greenUpper)
        green_mask = cv2.erode(green_mask, None, iterations=4)
        green_mask = cv2.dilate(green_mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        masks = [green_mask]#, red_mask, blue_mask]
        best_radius = 0
        best_x = 0
        best_y = 0
        center = None
        M = None
        for mask in masks:
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                if radius > best_radius:
                    try:
                        best_radius = radius
                        best_x = x
                        best_y = y
                        M = cv2.moments(c)
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    except:
                        center = None
                        best_radius = None
                        return center, best_radius
                        # only proceed if the radius meets a minimum size
                # if radius > 10:
                #     # draw the circle and centroid on the frame,
                #     # then update the list of tracked points
                #     cv2.circle(frame, (int(best_x), int(best_y)), int(best_radius),
                #         (0, 255, 255), 2)
                #     cv2.circle(frame, center, 5, (0, 0, 255), -1)
            # update the points queue
            pts.appendleft(center)

        return center, best_radius

    @classmethod
    def callback(self, image_msg, depth_msg):
        # Convert images to arrays
        img = self.cv_bridge.imgmsg_to_cv2(image_msg)[:, :, :3]
        img = img.copy()
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)

        _, r, _ = img.shape

        # Run inference
        # result = self.model.predict(source=img, conf=0.1, verbose=self.verbose)[0]
        center, radius = self.detect(img)
callback
        # Determine bounding box
        # out_img = cv2.rotate(img.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
        out_img = cv2.circle(img, (750, 1024), 15, (0,0,255), -1)

        if center is not None:
            print("I FOUND THE SHIT")
            #box = result.boxes[0].xyxy.squeeze()
            # out_img = cv2.rectangle(out_img, (int(box[3]), int(r - box[2])), (int(box[1]), int(r - box[0])), (255, 0, 0))
            #out_img = cv2.circle(img, (750, 1024), 5, (0,0,255), -1)
            #out_img = cv2.circle(img, center, int(radius), (0, 0, 255), -1)
            # Crop to center of box
            # lx = box[3] - box[1]
            # ly = box[2] - box[0]
            # x_offset = (lx - (lx / self.sub_box_scale) ) / 2
            # y_offset = (ly - (ly / self.sub_box_scale) ) / 2

            y1 = int(center[0] - radius/2)
            x1 = int(center[1] - radius/2)
            y2 = int(center[0] + radius/2)
            x2 = int(center[1] + radius/2)
            out_img = cv2.rectangle(out_img, (int(y2), int(x2)), (int(y1), int(x1)), (255, 0, 0),3)

            #print("Values", y1," ", x1," ", y2," ", x2)
            # Get median of depth
            depth_sample = depth[y1:y2, x1:x2]
            print(depth_sample[depth_sample > 0])
            # depth_sample = depth[center[0], center[1]]
            med_depth = np.median(depth_sample[depth_sample > 0])
            print("Med Depth", med_depth)
            #med_depth = depth[center[0], center[1]]
            if np.isnan(med_depth): return

            # Determine center of box in pixel coords
            px_x = center[0]#box[0] + lx / 2
            px_y = center[1]#box[1] + ly / 2

            # # Determine ball location in 3D
            x = med_depth
            y = -(px_x - self.intrinsic[0, 2]) / self.intrinsic[0, 0] * med_depth
            z = -(px_y - self.intrinsic[1, 2]) / self.intrinsic[1, 1] * med_depth

            # # Convert location to world frame
            x, y, z, _ = np.matmul(self.cam_to_world, np.array([x / 1000, y.item() / 1000 - 0.032, z.item() / 1000, 1]))

            # # Update position history
            # self.pos_hist.pop(0)
            # self.pos_hist.append((np.array([x, y, z]), rospy.get_time()))

            # # Filter position
            # x_filt, y_filt, z_filt = np.median([pos for pos, _ in self.pos_hist], axis=0)
            x_filt, y_filt, z_filt = x, y, z
            # # Update filtered position history
            # self.pos_hist_filt.pop(0)
            # self.pos_hist_filt.append((np.array([x_filt, y_filt, z_filt]), rospy.get_time()))

            if self.visualize:
                print("VIZUALIZINGGG")
                self.ball_marker.header.stamp = rospy.Time.now()
                self.ball_marker.pose.position.x = x_filt
                self.ball_marker.pose.position.y = y_filt
                self.ball_marker.pose.position.z = z_filt
                self.ball_pub.publish(self.ball_marker)
                print(x_filt)

            # if self.visualize:
            #     self.landing_marker.header.stamp = rospy.Time.now()
            #     self.landing_marker.pose.position = self.landing_pos
            #     self.landing_pub.publish(self.landing_marker)

            #     self.x_pub.publish(Float32(self.landing_pos.x))
            #     self.y_pub.publish(Float32(self.landing_pos.y))

            # # Publish the landing position
            # if self.landing_pos.x != np.inf:
            #     self.landing_pos_pub.publish(self.landing_pos)
        else:
            print("DID NOT FIND SHIT")
               
        if self.visualize:
            self.yolo_pub.publish(self.cv_bridge.cv2_to_imgmsg(out_img))
            # print("writing")
            # # cv2.imwrite("/home/student/Downloads/image.jpg", out_img)
            # out_img = imutils.resize(out_img, width=600)
            # cv2.imshow("image", out_img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
if __name__ == "__main__":
    track_fruit = TrackingNode()
    rospy.spin()