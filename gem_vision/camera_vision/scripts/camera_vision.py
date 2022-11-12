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

class ImageConverter:
    
    def __init__(self):
        self.node_name = "gem_vision"

        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)

        self.bridge = CvBridge()

        subcriber_rgb = message_filters.Subscriber('/front_single_camera/image_raw', Image)
        subcriber_left_depth = message_filters.Subscriber('/stereo/camera/left/image_raw', Image)
        subcriber_right_depth = message_filters.Subscriber('/stereo/camera/left/image_raw', Image)

        sync = message_filters.ApproximateTimeSynchronizer([subcriber_rgb, subcriber_left_depth, subcriber_right_depth], 10, 1)#同步时间戳，具体参数含义需要查看官方文档。
        sync.registerCallback(self.multi_callback)#执行反馈函数
        self.image_pub = rospy.Publisher("/front_single_camera/image_processed", Image, queue_size=1)

    def multi_callback(self, rgb, left_depth, right_depth):
        # Get rgb and depth image in cv2 format respectively
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            left_depth_frame = self.bridge.imgmsg_to_cv2(left_depth, "bgr8")
            right_depth_frame = self.bridge.imgmsg_to_cv2(right_depth, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        # print("rgb information", rgb_frame.shape)
        # print("Depth left information", left_depth_frame.shape)
        # print("Depth right information", right_depth_frame.shape)

        # ----------------- Imaging processing code starts here ----------------

        pub_image = np.copy(rgb_frame)

        image_frame = np.copy(rgb_frame)

        # cv2.imshow("Output2", left_depth_frame)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # cv2.imshow("Output2", right_depth_frame)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        detected_list = yolo_detect_image(image_frame)
        print("Detected Objects", detected_list)
        # print(cv2.__version__)
        
    
        # ----------------------------------------------------------------------
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

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

    