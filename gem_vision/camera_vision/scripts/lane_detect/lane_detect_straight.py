from lane_utils import *

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

# Lane Extrapolation
def find_lane_lines_formula(lines):
    xs = []
    ys = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            xs.append(x1)
            xs.append(x2)
            ys.append(y1)
            ys.append(y2)
    slope, intercept, r_value, p_value, std_err = stats.linregress(xs, ys)
    # Remember, a straight line is expressed as f(x) = Ax + b. Slope is the A, while intercept is the b
    return (slope, intercept)

def trace_lane_line(img, lines, top_y, mask_list, make_copy=True):
    A, b = find_lane_lines_formula(lines)
    vert = get_vertices_for_img(img, mask_list)
    img_shape = img.shape
    bottom_y = img_shape[0] - 1
    # y = Ax + b, therefore x = (y - b) / A
    x_to_bottom_y = (bottom_y - b) / A
    top_x_to_y = (top_y - b) / A 
    new_line = [int(x_to_bottom_y), int(bottom_y), int(top_x_to_y), int(top_y)]
    new_lines = [[new_line]]
    return draw_lines(img, new_lines, make_copy=make_copy), new_line

def trace_both_lane_lines(img, left_lane_lines, right_lane_lines, mask_list):
    vert = get_vertices_for_img(img, mask_list)
    region_top_left = vert[0][1]
    full_left_lane_img, full_left_lane = trace_lane_line(img, left_lane_lines, region_top_left[1], mask_list, make_copy=True)
    full_left_right_lanes_img, full_right_lane = trace_lane_line(full_left_lane_img, right_lane_lines, region_top_left[1], mask_list, make_copy=False)
    # image1 * α + image2 * β + λ
    # image1 and image2 must be the same shape.
    img_with_lane_weight =  cv2.addWeighted(img, 0.7, full_left_right_lanes_img, 0.3, 0.0)
    return img_with_lane_weight, full_left_lane, full_right_lane

def middle_lane_optimize(left_lane, right_lane, seperated_lane, img, mask_list):
    middle_points = []
    bottom_x = (left_lane[0] + right_lane[0]) / 2
    bottom_y = (left_lane[1] + right_lane[1]) / 2
    top_x = (left_lane[2] + right_lane[2]) / 2
    top_y = (left_lane[3] + right_lane[3]) / 2
    middle_lane = [float(bottom_x), float(bottom_y), float(top_x), float(top_y)]
    A, b = find_lane_lines_formula([[middle_lane]])
    vert = get_vertices_for_img(img, mask_list)
    # y = Ax + b, therefore x = (y - b) / A
    for y in range(int(top_y)+50, int(bottom_y)-50, 10):
        x = (y - b) / A
        point = [x, y]
        middle_points.append(point)
    middle_point = np.array(middle_points)
    middle_point = np.mean(middle_point, axis = 0)
    return [middle_point], A

def straight_lane_detector(frame, mask_list, threshold): 
    # 1. Convert image to HSL, isolate white lane, convert to gray scale
    white_lane, hsl_image = isolate_white_lane_hsl(frame, threshold[0], threshold[1])
    gray_scale_image = cv2.cvtColor(white_lane, cv2.COLOR_RGB2GRAY)
    # 2. blurr image with Guassian Kernel and generate Canny edge
    blurred_image = cv2.GaussianBlur(gray_scale_image, (5, 5), 0) 
    canny_image = cv2.Canny(blurred_image, 50, 150)
    # 3. The region of interest through * mask
    segmented_image, mask = region_of_interest(canny_image, mask_list)
    # 4. Hough Transform, generating parallel lanes
    hough_lines =  hough_transform(canny_img=segmented_image ,rho=1, theta=(np.pi/180) * 1, threshold=13, min_line_len=20, max_line_gap=10)
    try:
        # 4. Detect left and right lanes
        seperated_lane = separate_lines(hough_lines, frame)
        full_lane_image, left_lane, right_lane = trace_both_lane_lines(frame, seperated_lane[0], seperated_lane[1], mask_list)
        # 6. Generate middle lane
        middle_point, A = middle_lane_optimize(left_lane, right_lane, seperated_lane, full_lane_image, mask_list)
        img_with_lane_bbx = draw_points(full_lane_image, middle_point, [255, 0, 255])
    except :
        middle_point = []
        img_with_lane_bbx = frame
        A = 10000000

    return middle_point, img_with_lane_bbx, A