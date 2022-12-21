from lane_utils import *

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
def calculate_covirance(points, poly_points):
    distance_list = []
    distance_min_list = []
    for poly_point in poly_points:
        distance_list = []
        for point in points:
            poly_x = poly_point[0]
            poly_y = poly_point[1]
            point_x = point[0]
            point_y = point[1]
            distance = math.sqrt((poly_x - point_x)**2 + (poly_y - point_y)**2 )
            distance_list.append(distance)
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
    # Change numbers and position of points we used for cubic curve
    for poly_y in range(int(top_y) + 20, int(bottom_y) - 40, points_number):
        poly_x = poly(poly_y)
        poly_point= (int(poly_x), int(poly_y))
        poly_points.append(poly_point)
    return poly_points

# UKF Kalman Filter
def UKFKalman(current_state, current_variance, last_state, last_variance):
    current_state = [6 * current_state[0], 2 * current_state[1], current_state[2], current_state[3]]
    last_state = [6 * last_state[0], 2 * last_state[1], last_state[2], last_state[3]]
    kn = last_variance / (last_variance + current_variance) * 0.7
    merge_state = []
    for i in range(len(current_state)):
        merge_state.append(last_state[i] + kn * (current_state[i] - last_state[i]))
    variance = (1 - kn) * last_variance
    state = [1 / 6 * merge_state[0], 1 / 2 * merge_state[1], merge_state[2], merge_state[3]]
    return state, variance

# Cubic Curve Fitting
def fit_cubic_curve(points, points_number, img, top_y, last_state_info):
    xs = []
    ys = []
    for point in points:
        xs.append(point[0])
        ys.append(point[1])
    # param of three derivates function: change here
    current_state = np.polyfit(ys, xs, 3)
    # Calculate variance of current state
    poly_points = sample_poly_points(current_state, points_number, img, top_y)
    current_variance = calculate_covirance(points, poly_points)
    if len(last_state_info) == 0:
        state = current_state
        variance = current_variance
    else:
        # Insert Kalman Filter here
        last_state = last_state_info[0]
        last_variance = last_state_info[1]
        state, variance = UKFKalman(current_state, current_variance, last_state, last_variance)
    return state, variance

# Cubic Curve interpolation
def trace_lane_line(img, points_detected, top_y, points_number, last_state_info, point_color, make_copy=True):
    # Fit the cubic curve function
    state, variance = fit_cubic_curve(points_detected, points_number, img, top_y, last_state_info)
    # interpolate cubic curve according to y values
    poly_points = sample_poly_points(state, points_number, img, top_y)
    current_state = [state, variance]
    # print("poly_points", poly_points)
    return draw_points(img, poly_points, point_color, make_copy=make_copy), poly_points, current_state

# If one of lanes is not detected, make horizon shift to generate one
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
    current_state = []
    return draw_points(detected_lane_image, generated_points_list, point_color, make_copy=make_copy), generated_points_list, current_state

# Track left + right lanes and their variance 
def trace_both_lane_lines(img, points_detected, mask_list, last_state_info, points_number, make_copy=True):
    img_copy = np.copy(img) if make_copy else img
    vert = get_vertices_for_img(img_copy, mask_list)
    region_top_left = vert[0][1]
    left_lane_points = points_detected[0]
    right_lane_points = points_detected[1]
    left_detected_number = len(points_detected[0])
    right_detected_number = len(points_detected[1])
    # if left detected lane points is far less than right lane
    if len(left_lane_points) == 0:
        curve_lane_image, right_lane_points, curren_state_right_info= trace_lane_line(img_copy, right_lane_points, region_top_left[1], points_number, last_state_info[1], point_color = [0, 0, 255], make_copy=True)
        merged_lane_image, left_lane_points, curren_state_left_info = horizon_transform(curve_lane_image, right_lane_points, turning_direction = 0, point_color = [255, 123, 0], make_copy=True)
    # if right detected lane points is far less than left lane
    elif len(right_lane_points) == 0:
        curve_lane_image, left_lane_points, curren_state_left_info = trace_lane_line(img_copy, left_lane_points, region_top_left[1], points_number, last_state_info[0], point_color = [255, 0, 0], make_copy=True)
        merged_lane_image, right_lane_points, curren_state_right_info = horizon_transform(curve_lane_image, left_lane_points, turning_direction = 1, point_color = [0, 0, 255], make_copy=True)
    elif len(right_lane_points) == 0 and len(left_lane_points) == 0:
        merged_lane_image = img_copy
        curren_state_left_info = last_state_info[0]
        curren_state_right_info = last_state_info[1]
    else:
        # Track left lane
        curve_lane_image, left_lane_points, curren_state_left_info = trace_lane_line(img_copy, left_lane_points, region_top_left[1], points_number, last_state_info[0], point_color = [255, 0, 0], make_copy=True)
        # Track right lane
        merged_lane_image, right_lane_points, curren_state_right_info= trace_lane_line(curve_lane_image, right_lane_points, region_top_left[1], points_number, last_state_info[1], point_color = [0, 0, 255], make_copy=True)
    # Update Current state info [state, variance] for left and right lanes
    current_state_info = [curren_state_left_info, curren_state_right_info]
    return merged_lane_image, left_lane_points, right_lane_points, current_state_info

# Generate middle lane by avarage left and right lanes
def middle_lane_generator(left_lane, right_lane, current_state_info):
    middle_points = []
    for i in range(len(left_lane)):
        middle_point_x = (left_lane[i][0] + right_lane[i][0]) / 2
        middle_point_y = (left_lane[i][1] + right_lane[i][1]) / 2
        middle_points.append(([middle_point_x, middle_point_y]))
    return middle_points

# Sample points and classify left and right lane points
def detect_points(segmented_image, mask_list, img):
    # Sample points
    point_list = []
    for y in range(int(mask_list[1]), int(mask_list[0]) - 1, 5):
        for x in range(int(mask_list[2]), int(mask_list[3]) - 1):
            if segmented_image[y][x] > 253:
                point_list.append([x, y])
    # Classify left and right lanes 
    cluster, lane_exist = DBSCAN_lib(img.shape[1], point_list, eps = 30, MinPts = 4)
    if lane_exist == "left":
        points_detected = [cluster[0], []]
    elif lane_exist == "right":
        points_detected = [[], cluster[0]]
    elif lane_exist == "None":
        points_detected = [[], []]
    else:
        points_detected = [cluster[0], cluster[1]]
    return points_detected

# Preprocess the frame and detect left and right lanes' raw points
def preprocess(frame, low_threshold_list, high_threshold_list, mask_list):
    # 1. Convert image to HSL, isolate white lane, convert to gray scale
    white_lane, hsl_image = isolate_white_lane_hsl(frame, low_threshold_list, high_threshold_list)
    gray_scale_image = cv2.cvtColor(white_lane, cv2.COLOR_RGB2GRAY)
    # 2. blurr image with Guassian Kernel and generate Canny edge
    blurred_image = cv2.GaussianBlur(gray_scale_image, (5, 5), 0) 
    canny_image = cv2.Canny(blurred_image, 50, 150)
    # 3. The region of interest through * mask
    segmented_image, mask = region_of_interest(canny_image, mask_list)
    points_detected = detect_points(segmented_image, mask_list, frame)
    # 4. Hough Transform, detect lane points
    return points_detected, segmented_image

def turing_lane_detector(frame, last_state_info, mask_list, threshold): 
    # Step 5: points number of sampling on cubic curve
    points_number = 5
    #1. Preprocess the frame
    points_detected, segmented_image= preprocess(frame, threshold[0], threshold[1], mask_list)
    # 5. Generate left and right lanes
    # try:
    left_detected_points = draw_points(frame, points_detected[0], point_color=[233,134,0])
    right_detected_points = draw_points(left_detected_points, points_detected[1], point_color=[0,134,233])
    full_lane_image, left_lane, right_lane, current_state_info = trace_both_lane_lines(right_detected_points, points_detected, mask_list, last_state_info, points_number)
    # 6. Generate middle lane
    middle_points = middle_lane_generator(left_lane, right_lane, current_state_info)
    img_with_lane_bbx = draw_points(full_lane_image, middle_points, point_color=[0, 255, 0])
    # except :
    #     middle_points = []
    #     img_with_lane_bbx = bbx_frame
    #     current_state_info = [[], []]
    #     print("None Detected")
    return middle_points, img_with_lane_bbx, current_state_info, segmented_image
