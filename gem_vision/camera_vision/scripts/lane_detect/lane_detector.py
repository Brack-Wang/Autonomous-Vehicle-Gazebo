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
    for point in points:
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

# The region shoule be adjusted according to the application 
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

# Cubic Curve Fitting
def find_lane_lines_formula(lines):
    xs = []
    ys = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            xs.append(x1)
            xs.append(x2)
            ys.append(y1)
            ys.append(y2)
    # param of three derivates function: change here
    z = np.polyfit(ys, xs, 3) 
    poly = np.poly1d(z)
    return (poly)
    # Straight Line Fitting
    # Remember, a straight line is expressed as f(x) = Ax + b. Slope is the A, while intercept is the b
    # slope, intercept, r_value, p_value, std_err = stats.linregress(xs, ys)
    # return (slope, intercept)

def covirance_calculator(lines, poly_points):
    # print("lines", len(lines))
    # print("poly_point: ", len(poly_points))
    distance_list = []
    for poly_point in poly_points:
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
    distances = np.array(distance_list)
    distance_variance = np.var(distances)
    # print("distance_variance", distance_variance)
    return distance_variance



# Cubic Curve interpolation
def trace_lane_line(img, lines, top_y, points_number, point_color, make_copy=True):
    img_shape = img.shape
    bottom_y = img_shape[0] - 1
    # Fit the cubic curve function
    poly= find_lane_lines_formula(lines)
    # interpolate cubic curve according to y values
    poly_points = []
    for poly_y in range(int(top_y) + 50, int(bottom_y), points_number):
        poly_x = poly(poly_y)
        poly_point= (int(poly_x), int(poly_y))
        poly_points.append(poly_point)
    virance = covirance_calculator(lines, poly_points)

    return draw_points(img, poly_points, point_color, make_copy=make_copy), poly_points, virance
    # Straight Line interpolation
    # # y = Ax + b, therefore x = (y - b) / A
    # x_to_bottom_y = (bottom_y - b) / A
    # top_x_to_y = (top_y - b) / A 
    # new_line = [int(x_to_bottom_y), int(bottom_y), int(top_x_to_y), int(top_y)]
    # new_lines = [[new_line]]
    # return draw_lines(img, new_lines, make_copy=make_copy), new_line

def trace_both_lane_lines(img, left_lane_lines, right_lane_lines, mask_list, points_number, make_copy=True):
    img_copy = np.copy(img) if make_copy else img
    vert = get_vertices_for_img(img_copy, mask_list)
    region_top_left = vert[0][1]
    left_curve_lane_image, left_lane_points, virance_left = trace_lane_line(img_copy, left_lane_lines, region_top_left[1],points_number, point_color = [255, 0, 0], make_copy=True)
    right_right_curve_lane_image, right_lane_points, virance_right= trace_lane_line(left_curve_lane_image, right_lane_lines, region_top_left[1], points_number, point_color = [0, 0, 255], make_copy=True)
    # image1 * α + image2 * β + λ
    # image1 and image2 must be the same shape.
    # img_with_lane_weight =  cv2.addWeighted(img, 0.5, right_right_curve_lane_image, 0.3, 0.0)
    return right_right_curve_lane_image, left_lane_points, right_lane_points, virance_left, virance_right

def middle_lane_generator(left_lane, right_lane, virance_left, virance_right, lane_queue):
    middle_points_o = []
    middle_points_int = []
    middle_points = []
    for i in range(len(left_lane)):
        middle_point_x = (left_lane[i][0] + right_lane[i][0]) / 2
        middle_point_y = (left_lane[i][1] + right_lane[i][1]) / 2
        middle_point_o = [middle_point_x, middle_point_y]
        middle_points_o.append((middle_point_o))
        middle_point_int = (int(middle_point_x), int(middle_point_y))
        middle_points_int.append(middle_point_int)
    current_variance = (virance_left + virance_right) / 2
    
    if len(lane_queue) == 0:
        print("Lane is empty")
        middle_points = middle_points_o
        middle_variance = current_variance
    else:
        print("Lane is not empty")
        middle_points = []
        middle_points_int = []
        last_points = lane_queue[0]
        print("last_points", last_points)
        last_variance = lane_queue[1]
        kn = last_variance / (last_variance + current_variance)
        for i in range(len(middle_points_o)):
            middle_x = last_points[i][0] + kn * (middle_points_o[i][0] - last_points[i][0])
            middle_y = last_points[i][1] + kn * (middle_points_o[i][1] - last_points[i][1])
            middle_point = (float(middle_x), float(middle_y))
            middle_points.append(middle_point)
            middle_point_int = (int(middle_x), int(middle_y))
            middle_points_int.append(middle_point_int)
        middle_variance = (1 - kn) * lane_queue[1]
    return middle_points, middle_points_int, middle_variance


def lane_detector(frame, bbx_frame, lane_queue): 
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
    # left_b = 0
    # right_b = width 
    # left_t = 0
    # right_t = width
    mask_list = [bottom, top, left_b, right_b, left_t, right_t]


    # 1. Convert image to HSL, isolate white lane, convert to gray scale
    white_lane, hsl_image = isolate_white_lane_hsl(frame, low_threshold_list, high_threshold_list)
    gray_scale_image = cv2.cvtColor(white_lane, cv2.COLOR_RGB2GRAY)
    # cv2imshow(hsl_image,"hsl_image")
    # cv2imshow(white_lane, "white_lane")
    # cv2imshow(gray_scale_image, "gray_scale_image")

    # 2. blurr image with Guassian Kernel and generate Canny edge
    blurred_image = gaussian_blur(gray_scale_image, kernel_size=5)
    canny_image = cv2.Canny(blurred_image, 50, 150)
    # cv2imshow(blurred_image,"blurred_image")
    # cv2imshow(canny_image,"canny_image")

    # 3. The region of interest through * mask
    segmented_image, mask = region_of_interest(canny_image, mask_list)
    # cv2imshow(mask,"mask")
    # cv2imshow(segmented_image,"segmented_image")

    # 4. Hough Transform, detect lane points

    hough_lines =  hough_transform(canny_img=segmented_image ,rho=1, theta=(np.pi/180) * 1, threshold=13, min_line_len=20, max_line_gap=10)
    # 5. Generate left and right lanes
    # try:
    seperated_lane = separate_lines(hough_lines, frame)
    different_color_lane_image = color_lanes(frame, seperated_lane[0], seperated_lane[1])
    # cv2imshow(different_color_lane_image, "different_color_lane_image")

    full_lane_image, left_lane, right_lane, virance_left, virance_right = trace_both_lane_lines(different_color_lane_image, seperated_lane[0], seperated_lane[1], mask_list, points_number = 10)

    # print("left_lane_full: ", len(left_lane))
    # print("right_lane_full: ", len(right_lane)) 
    # cv2imshow(full_lane_image, "full_lane_image")

    # 6. Generate middle lane
    middle_points_float, middle_points_int, virance_middle = middle_lane_generator(left_lane, right_lane, virance_left, virance_right, lane_queue)
    img_with_lane_bbx = draw_points(full_lane_image, middle_points_int, point_color=[0, 255, 0])
    # img_with_lane_bbx = draw_lines(bbx_frame, [[middle_lane]])

    # print("middle_lane", middle_lane)
    # cv2imshow(img_with_lines)
    # except :
    #     middle_lane = []
    #     img_with_lane_bbx = bbx_frame
    #     print("None Detected")


    return middle_points_float, img_with_lane_bbx, virance_middle
