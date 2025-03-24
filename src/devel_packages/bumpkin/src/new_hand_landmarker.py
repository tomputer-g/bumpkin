import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import message_filters
#!/usr/bin/env python3


class RealSenseViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.color_sub], 10, 5)
        self.ts.registerCallback(self.callback)

    def callback(self, depth_msg, color_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            color_image_resized = cv2.resize(color_image, (848, 480))
            combined_image = cv2.hconcat([color_image_resized, depth_colormap])
            cv2.imshow('Combined Image', combined_image)
            # cv2.imshow('Depth Colormap', depth_colormap)
            # cv2.imshow('Color Image', color_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('realsense_viewer', anonymous=True)
    viewer = RealSenseViewer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()