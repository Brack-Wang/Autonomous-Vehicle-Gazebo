#importing some useful packages
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from scipy import stats
import cv2
import math

def cv2imshow(frame, frame_name):
    cv2.imshow(frame_name, frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def draw_lines(img, lines, color, thickness=4, make_copy=True):
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

def color_lanes(img, left_lane_lines, right_lane_lines, left_lane_color=[255, 255, 0], right_lane_color=[0, 255, 255]):
    left_colored_img = draw_lines(img, left_lane_lines, color=left_lane_color, make_copy=True)
    right_colored_img = draw_lines(left_colored_img, right_lane_lines, color=right_lane_color, make_copy=False)
    return right_colored_img

def gaussian_blur(grayscale_img, kernel_size=3, ):
    return cv2.GaussianBlur(grayscale_img, (kernel_size, kernel_size), 0) 

def isolate_white_lane_hsl(frame, low_threshold_list, high_threshold_list):
    # Caution - OpenCV encodes the data in ***HLS*** format
    hsl_image = cv2.cvtColor(frame, cv2.COLOR_RGB2HLS)
    low_threshold = np.array(low_threshold_list, dtype=np.uint8)
    high_threshold = np.array(high_threshold_list, dtype=np.uint8)  
    white_mask = cv2.inRange(hsl_image, low_threshold, high_threshold)
    white_lane = cv2.bitwise_and(frame, frame, mask=white_mask)
    return white_lane, hsl_image

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

# Separating Left And Right Lanes
def separate_lines(lines, img):
    img_shape = img.shape
    middle_x = img_shape[1] / 2
    left_lane_lines = []
    right_lane_lines = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            dx = x2 - x1 
            if dx == 0:
                #Discarding line since we can't gradient is undefined at this dx
                continue
            dy = y2 - y1
            # Similarly, if the y value remains constant as x increases, discard line
            if dy == 0:
                continue
            slope = dy / dx
            # This is pure guess than anything... 
            # but get rid of lines with a small slope as they are likely to be horizontal one
            epsilon = 0.1
            if abs(slope) <= epsilon:
                continue
            if slope < 0 and x1 < middle_x and x2 < middle_x:
                # Lane should also be within the left hand side of region of interest
                left_lane_lines.append([[x1, y1, x2, y2]])
            elif x1 >= middle_x and x2 >= middle_x:
                # Lane should also be within the right hand side of region of interest
                right_lane_lines.append([[x1, y1, x2, y2]])
    return left_lane_lines, right_lane_lines

# Calculate min distance for each poly_points to all points in hough transform(lines) and get variance
def calculate_covirance(lines, poly_points):
    distance_list = []
    distance_min_list = []
    for poly_point in poly_points:
        distance_list = []
        for line in lines:
            poly_x = poly_point[0]
            poly_y = poly_point[1]
            line_x1 = line[0][0]
            line_y1 = line[0][1]
            line_x2 = line[0][2]
            line_y2 = line[0][3]
            distance_1 = math.sqrt((poly_x - line_x1)**2 + (poly_y - line_y1)**2 )
            distance_2 = math.sqrt((poly_x - line_x2)**2 + (poly_y - line_y2)**2 )
            distance_list.append(distance_1)
            distance_list.append(distance_2)
        distance_min = min(distance_list)
        distance_min_list.append(distance_min)
    distances = np.array(distance_min_list)
    distance_variance = np.var(distances)
    # print("distance_variance", distance_variance)
    return distance_variance

def sample_poly_points(current_state, points_number, img, top_y):
    poly = np.poly1d(current_state)
    bottom_y = img.shape[0] - 1
    poly_points = []
    for poly_y in range(int(top_y) + 50, int(bottom_y), points_number):
        poly_x = poly(poly_y)
        poly_point= (int(poly_x), int(poly_y))
        poly_points.append(poly_point)
    return poly_points

# UKF Kalman Filter
def UKFKalman(current_state, current_variance, last_state, last_variance):
    current_state = [6 * current_state[0], 2 * current_state[1], current_state[2], current_state[3]]
    last_state = [6 * last_state[0], 2 * last_state[1], last_state[2], last_state[3]]
    # print("current_state", current_state)
    # print("last_state", last_state)
    kn = last_variance / (last_variance + current_variance * 0.1)
    merge_state = []
    for i in range(len(current_state)):
        merge_state.append(last_state[i] + kn * (current_state[i] - last_state[i]))
    # print("merge_state", merge_state)
    variance = (1 - kn) * last_variance
    state = [1 / 6 * merge_state[0], 1 / 2 * merge_state[1], merge_state[2], merge_state[3]]
    # print("state:", state)
    return state, variance

# Cubic Curve Fitting
def fit_cubic_curve(lines, points_number, img, top_y, last_state_info):
    xs = []
    ys = []
    # print("lines", lines)
    for line in lines:
        for x1, y1, x2, y2 in line:
            xs.append(x1)
            xs.append(x2)
            ys.append(y1)
            ys.append(y2)
    # param of three derivates function: change here
    current_state = np.polyfit(ys, xs, 3)
    # Calculate variance of current state
    poly_points = sample_poly_points(current_state, points_number, img, top_y)
    current_variance = calculate_covirance(lines, poly_points)
    # print("last_state_info", last_state_info)
    if len(last_state_info) == 0:
        state = current_state
        variance = current_variance
    else:
        # Insert Kalman Filter here
        # print("last_state_info", last_state_info)
        last_state = last_state_info[0]
        last_variance = last_state_info[1]
        state, variance = UKFKalman(current_state, current_variance, last_state, last_variance)
    return state, variance

# Cubic Curve interpolation
def trace_lane_line(img, lines, top_y, points_number, last_state_info, point_color, make_copy=True):
    # Fit the cubic curve function
    state, variance = fit_cubic_curve(lines, points_number, img, top_y, last_state_info)
    # interpolate cubic curve according to y values
    poly_points = sample_poly_points(state, points_number, img, top_y)
    current_state = [state, variance]
    # print("poly_points", poly_points)
    return draw_points(img, poly_points, point_color, make_copy=make_copy), poly_points, current_state

def horizon_transform(detected_lane_image, detected_lane_points, turning_direction, point_color, make_copy=True):
    generated_points_list = []
    detected_first_point =  detected_lane_points[0]
    if turning_direction == 0:
        generated_first_point = [detected_first_point[0] - 400, detected_first_point[1]]
    else:
        generated_first_point = [detected_first_point[0] + 400, detected_first_point[1]]
    generated_points_list.append(generated_first_point)

    for i in range(len(detected_lane_points) - 1):
        distance_x = detected_lane_points[i+1][0] - detected_lane_points[i][0]
        distance_y = detected_lane_points[i+1][1] - detected_lane_points[i][1]
        left_point = [generated_points_list[i][0] + distance_x, generated_points_list[i][1] + distance_y]
        generated_points_list.append(left_point)
    # print("\nright_lane_points", detected_lane_points,"\nleft_lane_points", generated_points_list)
    # return draw_points(right_right_curve_lane_image, poly_points, point_color, make_copy=True)
    current_state = []
    return draw_points(detected_lane_image, generated_points_list, point_color, make_copy=make_copy), generated_points_list, current_state

# Track left + right lanes and their variance 
def trace_both_lane_lines(img, seperated_lane, mask_list, last_state_info, points_number, make_copy=True):
    img_copy = np.copy(img) if make_copy else img
    left_lane_lines = seperated_lane[0]
    right_lane_lines = seperated_lane[1]
    vert = get_vertices_for_img(img_copy, mask_list)
    region_top_left = vert[0][1]
    left_detected_number = len(seperated_lane[0])
    right_detected_number = len(seperated_lane[1])
    # if left detected lane points is far less than right lane
    if left_detected_number == 0 and right_detected_number != 0:
        curve_lane_image, right_lane_points, curren_state_right_info= trace_lane_line(img_copy, right_lane_lines, region_top_left[1], points_number, last_state_info[1], point_color = [0, 0, 255], make_copy=True)
        merged_lane_image, left_lane_points, curren_state_left_info = horizon_transform(curve_lane_image, right_lane_points, turning_direction = 0, point_color = [255, 123, 0], make_copy=True)
    # if right detected lane points is far less than left lane
    elif right_detected_number == 0 and left_detected_number != 0:
        curve_lane_image, left_lane_points, curren_state_left_info = trace_lane_line(img_copy, left_lane_lines, region_top_left[1], points_number, last_state_info[0], point_color = [255, 0, 0], make_copy=True)
        merged_lane_image, right_lane_points, curren_state_right_info = horizon_transform(curve_lane_image, left_lane_points, turning_direction = 1, point_color = [0, 0, 255], make_copy=True)
    else:
        # Track left lane
        curve_lane_image, left_lane_points, curren_state_left_info = trace_lane_line(img_copy, left_lane_lines, region_top_left[1], points_number, last_state_info[0], point_color = [255, 0, 0], make_copy=True)
        # Track right lane
        merged_lane_image, right_lane_points, curren_state_right_info= trace_lane_line(curve_lane_image, right_lane_lines, region_top_left[1], points_number, last_state_info[1], point_color = [0, 0, 255], make_copy=True)
    # Update Current state info [state, variance] for left and right lanes
    current_state_info = [curren_state_left_info, curren_state_right_info]
    return merged_lane_image, left_lane_points, right_lane_points, current_state_info

# Generate middle lane by avarage left and right lanes
def middle_lane_generator(left_lane, right_lane):
    middle_points = []
    for i in range(len(left_lane)):
        middle_point_x = (left_lane[i][0] + right_lane[i][0]) / 2
        middle_point_y = (left_lane[i][1] + right_lane[i][1]) / 2
        middle_points.append(([middle_point_x, middle_point_y]))
    return middle_points

# Preprocess the frame and detect left and right lanes' raw points
def preprocess(frame, low_threshold_list, high_threshold_list, mask_list):
    # 1. Convert image to HSL, isolate white lane, convert to gray scale
    white_lane, hsl_image = isolate_white_lane_hsl(frame, low_threshold_list, high_threshold_list)
    gray_scale_image = cv2.cvtColor(white_lane, cv2.COLOR_RGB2GRAY)
    # 2. blurr image with Guassian Kernel and generate Canny edge
    blurred_image = gaussian_blur(gray_scale_image, kernel_size=5)
    canny_image = cv2.Canny(blurred_image, 50, 150)
    # 3. The region of interest through * mask
    segmented_image, mask = region_of_interest(canny_image, mask_list)
    # 4. Hough Transform, detect lane points
    hough_lines =  hough_transform(canny_img=segmented_image ,rho=1, theta=(np.pi/180) * 1, threshold=13, min_line_len=20, max_line_gap=10)
    # cv2imshow(hsl_image,"hsl_image")
    # cv2imshow(white_lane, "white_lane")
    # cv2imshow(gray_scale_image, "gray_scale_image")
    # cv2imshow(blurred_image,"blurred_image")
    # cv2imshow(canny_image,"canny_image")
    # cv2imshow(mask,"mask")
    # cv2imshow(segmented_image,"segmented_image")
    return hough_lines

def lane_detector(frame, bbx_frame, last_state_info): 
    frame_shape = frame.shape
    height = frame_shape[0]
    width = frame_shape[1]
    # cv2imshow(frame)
    # print(frame_shape)

    # Super Parameter
    # Step 1: Lower threshold and higher threshold of HSL filter
    # Lower value equivalent pure HSL is [30, 45, 15]  / [0, 200, 0] /  
    # Higher value equivalent pure HSL is [360, 100, 100]  / [180, 255, 255] / 
    low_threshold_list =[0, 200, 0] 
    high_threshold_list = [180, 255, 255]
    # Step 3: 4 points position of Mask 
    bottom = height - 1
    top = height * 3 / 5
    left_b = 30 
    right_b = width - 30
    left_t = left_b + 200
    right_t = right_b - 200
    mask_list = [bottom, top, left_b, right_b, left_t, right_t]
    # Step 5: points number of sampling on cubic curve
    points_number = 10

    #1. Preprocess the frame
    hough_lines = preprocess(frame, low_threshold_list, high_threshold_list, mask_list)
    
    # 5. Generate left and right lanes
    try:
        seperated_lane = separate_lines(hough_lines, frame)
        different_color_lane_image = color_lanes(bbx_frame, seperated_lane[0], seperated_lane[1])
        # cv2imshow(different_color_lane_image, "different_color_lane_image")

        full_lane_image, left_lane, right_lane, current_state_info = trace_both_lane_lines(different_color_lane_image, seperated_lane, mask_list, last_state_info, points_number)

        # print("left_lane_full: ", len(left_lane))
        # print("right_lane_full: ", len(right_lane)) 
        # cv2imshow(full_lane_image, "full_lane_image")

        # 6. Generate middle lane
        middle_points = middle_lane_generator(left_lane, right_lane)
        img_with_lane_bbx = draw_points(full_lane_image, middle_points, point_color=[0, 255, 0])
        # img_with_lane_bbx = draw_lines(bbx_frame, [[middle_lane]])

        # print("middle_lane", middle_lane)
        # cv2imshow(img_with_lines)
    except :
        middle_points = []
        img_with_lane_bbx = bbx_frame
        current_state_info = [[], []]
        print("None Detected")

    return middle_points, img_with_lane_bbx, current_state_info
