#! /usr/bin/env python
from __future__ import print_function

import sys
import copy
import time
import rospy
import math
import tf

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import message_filters
sys.path.append("./src/gem_vision/camera_vision/scripts/Detector/")
from yolo_detect_image import yolo_detect_image
sys.path.append("./src/gem_vision/camera_vision/scripts/lane_detect/")
from lane_detector import lane_detector
from camera_vision.msg import Detected_info

# Use this code to save sub video of rosbag you want
class ImageConverter:
    def __init__(self):
        self.node_name = "gem_vision"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        # Subscribe camera rgb and depth information
        depth_img_topic = rospy.get_param('depth_info_topic','/zed2/zed_node/depth/depth_registered')
        self.depth_img_sub = message_filters.Subscriber(depth_img_topic,Image)
        self.subcriber_rgb = message_filters.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image)
        sync = message_filters.ApproximateTimeSynchronizer([self.subcriber_rgb, self.depth_img_sub], 10, 1)
        sync.registerCallback(self.multi_callback)
        self.fileNameIndex = 0

    def cv2imshow(self, frame, frame_name, mode):
        cv2.imshow(frame_name, frame)
        if mode == 0:
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        elif mode == 1:
            cv2.waitKey(1)        

    def isolate_white_lane_hsl(self, frame, low_threshold_list, high_threshold_list):
        # Caution - OpenCV encodes the data in ***HLS*** format
        hsl_image = cv2.cvtColor(frame, cv2.COLOR_RGB2HLS)
        low_threshold = np.array(low_threshold_list, dtype=np.uint8)
        high_threshold = np.array(high_threshold_list, dtype=np.uint8)  
        white_mask = cv2.inRange(hsl_image, low_threshold, high_threshold)
        white_lane = cv2.bitwise_and(frame, frame, mask=white_mask)
        return white_lane, hsl_image

    def multi_callback(self, rgb, depth):
        fileNameIndex = 0
        # Get rgb and depth image in cv2 format respectively
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            depth_frame = self.bridge.imgmsg_to_cv2(depth, "32FC1")
            # print("rgb_frame", rgb_frame.shape)
            # print("depth_frame", depth_frame.shape)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        # ----------------- Imaging processing code starts here ----------------\
        self.fileNameIndex = self.fileNameIndex + 1
        if self.fileNameIndex%10 == 0:
            cv2.imwrite("./src/gem_vision/camera_vision/scripts/lane_detect/targetImg/2/" + str(self.fileNameIndex) + ".jpg", rgb_frame[240: 720, :])

    def cleanup(self):
        print ("Shutting down vision node.")
        cv2.destroyAllWindows()

def main(args):
    try:
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down vision node.")
        cv2.destryAllWindows()

if __name__ == '__main__':
    main(sys.argv)


    