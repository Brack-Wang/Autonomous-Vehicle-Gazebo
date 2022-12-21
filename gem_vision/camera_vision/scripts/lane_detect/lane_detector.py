#importing some useful packages
from lane_utils import *

from lane_detect_straight import straight_lane_detector
from lane_detect_turn import turing_lane_detector
from ssim_compare import img_compare

# Three different methos to detect lane: straight methods, turning methods and image match
def lane_detector(frame, last_state_info, last_ssim_info, frame_counter, souce_path): 
    # 1. straight lane detector
    height = frame.shape[0]
    width = frame.shape[1]
    bottom = height - 30
    top = height * 1 / 2
    left_b = 250 
    right_b = width - 250
    left_t = left_b + 300
    right_t = right_b - 300
    mask = [bottom, top, left_b, right_b, left_t, right_t]
    
    low_threshold_list = [0, 140, 0]
    high_threshold_list = [180, 255, 255]
    threshold = [low_threshold_list, high_threshold_list]


    middle_point, img_with_lane_bbx, A = straight_lane_detector(frame, mask, threshold)
    current_state_info = last_state_info

    # 2. Image Match Method
    # signal = img_compare(frame, threshold, last_ssim_info, frame_counter, souce_path)
    # angle = 10000000
    # if A > 2 and A < 22:
    #     signal = 0
    #     front_point = [width / 2, height]
    #     angle = (middle_point[0][1] - front_point[1]) / (middle_point[0][0] -front_point[0])
    # last_state_info = [[], []]

    # 3. Turning lane detector
    top = height * 2 / 3
    left_b = 150 
    right_b = width - 150
    left_t = left_b + 400
    right_t = right_b - 400
    mask = [bottom, top, left_b, right_b, left_t, right_t]

    middle_points, img_with_lane_bbx, current_state_info, segmented_image = turing_lane_detector(frame, last_state_info, mask, threshold)
    signal = 0
    angle = 0
    return middle_points, img_with_lane_bbx, current_state_info, last_ssim_info, signal, angle, segmented_image
