#! /usr/bin/env python
from __future__ import print_function

import sys
import copy
import time
import rospy
import rospkg

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
from camera_vision.msg import DetectBox

class ImageConverter:
    def __init__(self):
        self.node_name = "gem_vision"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        # Subscribe camera rgb and depth information
        subcriber_rgb = message_filters.Subscriber('/front_single_camera/image_raw', Image)
        subcriber_left_depth = message_filters.Subscriber('/stereo/camera/left/image_raw', Image)
        subcriber_right_depth = message_filters.Subscriber('/stereo/camera/left/image_raw', Image)
        sync = message_filters.ApproximateTimeSynchronizer([subcriber_rgb, subcriber_left_depth, subcriber_right_depth], 10, 1)
        sync.registerCallback(self.multi_callback)
        # Publish Boudingbox information of objects
        self.image_pub = rospy.Publisher("/front_single_camera/object_detection", DetectBox, queue_size=1)

    def multi_callback(self, rgb, left_depth, right_depth):
        # Get rgb and depth image in cv2 format respectively
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            left_depth_frame = self.bridge.imgmsg_to_cv2(left_depth, "bgr8")
            right_depth_frame = self.bridge.imgmsg_to_cv2(right_depth, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        # ----------------- Imaging processing code starts here ----------------\
        # Object Detection with Yolov3 through OpenCV
        detected_list, bbx_frame = yolo_detect_image(rgb_frame)
        print("Detected Objects", detected_list)
        cv2.imshow("Output", bbx_frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        middle_lane, img_with_lane_bbxs = lane_detector(rgb_frame, bbx_frame)
        print("middle_lane", middle_lane)
        cv2.imshow("Output", img_with_lane_bbxs)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        detectBox = DetectBox()
        for i in range(len(middle_lane)):
            detectBox.middle_lane.append(middle_lane[i])
        for i in  range(len(detected_list)):
            detectBox.center_x.append(detected_list[i][0])
            detectBox.center_y.append(detected_list[i][1])
            detectBox.width.append(detected_list[i][2])
            detectBox.height.append(detected_list[i][3])
            detectBox.classId.append(detected_list[i][4])
            detectBox.confidence.append(detected_list[i][5])
        # ----------------------------------------------------------------------
        self.image_pub.publish(detectBox)

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

    