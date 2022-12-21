import cv2
import numpy as np
import os
import sys
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy import stats
from Dbscan import MYDBSCAN, DBSCAN_lib
from skimage.metrics import structural_similarity as ssim
from sklearn.cluster import DBSCAN

def cv2imshow(frame, frame_name, mode):
    cv2.imshow(frame_name, frame)
    if mode == 0:
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    elif mode == 1:
        cv2.waitKey(1)

def draw_lines(img, lines, color=[255, 0, 0], thickness=10, make_copy=True):
    # Copy the passed image
    img_copy = np.copy(img) if make_copy else img
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img_copy, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
    return img_copy

def draw_points(img, points, point_color, make_copy=True):
    point_size = 1
    thickness=5
    img_copy = np.copy(img) if make_copy else img
    poins_int = []
    for point in points:
        poins_int.append((int(point[0]), int(point[1])))
    for point in poins_int:
        cv2.circle(img_copy, point, point_size, point_color, thickness)
    return img_copy

def color_lanes(img, left_lane_lines, right_lane_lines, left_lane_color=[255, 0, 0], right_lane_color=[0, 0, 255]):
    left_colored_img = draw_lines(img, left_lane_lines, color=left_lane_color, make_copy=True)
    right_colored_img = draw_lines(left_colored_img, right_lane_lines, color=right_lane_color, make_copy=False)
    return right_colored_img

# The mask coordinates of the region of interest, which shoule be adjusted according to the application 
def get_vertices_for_img(img, mask_list):
    bottom = mask_list[0]
    top = mask_list[1]
    left_b = mask_list[2]
    right_b = mask_list[3]
    left_t = mask_list[4]
    right_t = mask_list[5]
    # set 4 position of mask
    vert = None
    region_bottom_left = (left_b , bottom)
    region_bottom_right = (right_b, bottom)
    region_top_left = (left_t, top)
    region_top_right = (right_t, top)
    vert = np.array([[region_bottom_left , region_top_left, region_top_right, region_bottom_right]], dtype=np.int32)
    return vert

def region_of_interest(img, mask_list):
    #defining a blank mask to start with
    mask = np.zeros_like(img)        
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255 
    vert = get_vertices_for_img(img, mask_list)    
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vert, ignore_mask_color)
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image, mask

def hough_transform(canny_img, rho, theta, threshold, min_line_len, max_line_gap):
    return cv2.HoughLinesP(canny_img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

def isolate_white_lane_hsl(frame, low_threshold_list, high_threshold_list):
    # Caution - OpenCV encodes the data in ***HLS*** format
    hsl_image = cv2.cvtColor(frame, cv2.COLOR_RGB2HLS)
    low_threshold = np.array(low_threshold_list, dtype=np.uint8)
    high_threshold = np.array(high_threshold_list, dtype=np.uint8)  
    white_mask = cv2.inRange(hsl_image, low_threshold, high_threshold)
    white_lane = cv2.bitwise_and(frame, frame, mask=white_mask)
    return white_lane, hsl_image