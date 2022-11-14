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
from camera_vision.msg import Boudingbox

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
        self.image_pub = rospy.Publisher("/front_single_camera/object_detection", Boudingbox, queue_size=1)

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
        image_frame = np.copy(rgb_frame)
        detected_list = yolo_detect_image(image_frame)
        print("Detected Objects", detected_list)
        bbx = Boudingbox()
        for i in  range(len(detected_list)):
            bbx.center_x.append(detected_list[i][0])
            bbx.center_y.append(detected_list[i][1])
            bbx.width.append(detected_list[i][2])
            bbx.height.append(detected_list[i][3])
            bbx.classId.append(detected_list[i][4])
            bbx.confidence.append(detected_list[i][5])
        # ----------------------------------------------------------------------
        self.image_pub.publish(bbx)

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

    