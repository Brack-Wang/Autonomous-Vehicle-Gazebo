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
from camera_vision.msg import Detected_msg
from collections import deque


def cv2imshow(frame, frame_name, mode):
    cv2.namedWindow(frame_name, 0)
    cv2.resizeWindow(frame_name, 2000, 1000)
    cv2.imshow(frame_name, frame)
    if mode == 0:
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    elif mode == 1:
        cv2.waitKey(1)

# Get the depth information and calculate the distance for detected object
def calculate_object_distance(detected_list, depth_frame, camera_info, bbx_frame):
    camera_coordinate_list = []
    # Camera Intrinsic Information
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]
    inv_fx = 1. / fx
    inv_fy = 1. / fy
    for i in range(len(detected_list)):
        left_top_x = detected_list[i][0]
        left_top_y = detected_list[i][1]
        right_down_x = detected_list[i][2]
        right_down_y = detected_list[i][3]
        classId = detected_list[i][4]
        confidence = detected_list[i][5]
        center_y = (left_top_y + right_down_y) / 2
        center_x = (left_top_x + right_down_x) / 2
        width = right_down_x - left_top_x
        height = right_down_y - left_top_y
        # Crop detected objects in depth img
        detected_depth = np.array(depth_frame[left_top_y : right_down_y, left_top_x : right_down_x])
        detected_depth = detected_depth[np.where(detected_depth > 0)]
        detected_depth = detected_depth[np.where(detected_depth < 100)]
        # Transform from pixel coordinate to camera coordinate
        if detected_depth.shape[0] == 0:
            camera_x = -1
            camera_y = -1
            camera_z = -1
            distance = -1
            camera_coordinate = [distance, camera_x, camera_y, classId, confidence, camera_z]
        else:
            camera_z = int(np.mean(detected_depth))
            camera_x = ((center_x + width / 2) - cx) * camera_z * inv_fx
            camera_y = ((center_y + height / 2) - cy) * camera_z * inv_fy
            distance = math.sqrt(camera_x ** 2  + camera_z ** 2)
            camera_coordinate = [distance, camera_x, camera_y, classId, confidence, camera_z]
        camera_coordinate_list.append(camera_coordinate)
    return camera_coordinate_list, bbx_frame

# Get the depth information and calculate the distance for detected lane
def calculate_lane_distance(middle_lane, depth_frame, camera_info, bbx_frame):
    camera_coordinate_list = []
    # Camera Intrinsic Information
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]
    inv_fx = 1. / fx
    inv_fy = 1. / fy
    for i in range(len(middle_lane)):
        center_y = middle_lane[i][0]
        center_x = middle_lane[i][1]
        width = 30
        height = 30
        # Crop detected objects in depth img
        detected_depth = np.array(depth_frame[int(center_y - height / 2) : int(center_y + height / 2), int(center_x - width / 2): int(center_x + width / 2)])
        detected_depth = detected_depth[np.where(detected_depth > 0)]
        detected_depth = detected_depth[np.where(detected_depth < 100)]
        # Transform from pixel coordinate to camera coordinate
        if detected_depth.shape[0] == 0:
            camera_x = -1
            camera_y = -1
            camera_z = -1
            distance = -1
            camera_coordinate = [distance, camera_x, camera_y, camera_z]
        else:
            camera_z = int(np.mean(detected_depth))
            camera_x = ((center_x + width / 2) - cx) * camera_z * inv_fx
            camera_y = ((center_y + height / 2) - cy) * camera_z * inv_fy
            distance = math.sqrt(camera_x ** 2 + camera_y ** 2 + camera_z ** 2)
            camera_coordinate = [distance, camera_x, camera_y, camera_z]
        camera_coordinate_list.append(camera_coordinate)
        # print("distance,", distance)
    return camera_coordinate_list, bbx_frame