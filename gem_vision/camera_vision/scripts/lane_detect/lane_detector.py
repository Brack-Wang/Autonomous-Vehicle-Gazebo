#importing some useful packages
from lane_utils import *

from lane_detect_straight import straight_lane_detector
from lane_detect_turn import turing_lane_detector
from ssim_compare import img_compare


def lane_detector(frame, last_state_info, last_ssim_info, frame_counter, souce_path): 
    height = frame.shape[0]
    width = frame.shape[1]
    bottom = height - 1
    top = height * 1 / 2
    left_b = 250 
    right_b = width - 250
    left_t = left_b + 300
    right_t = right_b - 300
    mask = [bottom, top, left_b, right_b, left_t, right_t]
    
    low_threshold_list = [0, 140, 0]
    high_threshold_list = [180, 255, 255]
    threshold = [low_threshold_list, high_threshold_list]


    middle_points, img_with_lane_bbx, A = straight_lane_detector(frame, mask, threshold)
    current_state_info = last_state_info
    signal = img_compare(frame, threshold, last_ssim_info, frame_counter, souce_path)
    if A > 2 and A < 22:
        signal = 0


    # last_state_info = [[], []]
    # if len(middle_points) == 0:
    # top = height * 2 / 3
    # left_b = 100 
    # right_b = width - 300
    # left_t = left_b + 100
    # right_t = right_b - 100
    # mask = [bottom, top, left_b, right_b, left_t, right_t]

    # middle_points, img_with_lane_bbx, current_state_info = turing_lane_detector(frame, last_state_info, mask, threshold)

    return middle_points, img_with_lane_bbx, current_state_info, last_ssim_info, signal
