from lane_utils import *

def preprocess(frame, threshold):
    white_lane, hsl_image = isolate_white_lane_hsl(frame, threshold[0], threshold[1])
    gray_scale_image = cv2.cvtColor(white_lane, cv2.COLOR_RGB2GRAY)
    blurred_image = cv2.GaussianBlur(gray_scale_image, (5, 5), 0) 
    return blurred_image

# Use SSIM to determine the similarity of two image
def img_compare(frame, threshold, last_ssim_info, frame_counter, souce_path):
        ssim_dict = dict()
        ssim_dict = {"1": 0, "1c": 0, "2": 0,"2c": 0}
        ssim_list = []
        source_frame = preprocess(frame, threshold)
        PATH = souce_path + "/lane_detect/targetImg"
        for number in range(1,3,1):
            file_path = PATH+'/'+str(number) +'/'
            for file in os.listdir(file_path):
                target_img = cv2.imread(file_path+'/'+file)
                target_img = preprocess(target_img, threshold)
                SSIM = ssim(target_img, source_frame[240: 720, :])
                current_ssim = ssim_dict[str(number)] + SSIM
                ssim_dict.update({str(number): current_ssim}) 
                current_number = ssim_dict[str(number)+"c"] + 1
                ssim_dict.update({str(number)+"c": current_number}) 
        for i in range(1,3,1):
            ssim_a = ssim_dict[str(i)]
            number_a = ssim_dict[str(i)+"c"]
            if number_a != 0:
                ssim_average = ssim_a / number_a
            else:
                ssim_average = 0
            ssim_list.append(ssim_average)
        # print(ssim_list)
        if (frame_counter > 5):
            last_ssim_info[0].append(ssim_list[0])
            last_ssim_info[0].popleft()
            last_ssim_info[1].append(ssim_list[1])
            last_ssim_info[1].popleft()
        else:
            last_ssim_info[0].append(ssim_list[0])
            last_ssim_info[1].append(ssim_list[1])
        first_turn = np.array(last_ssim_info[0]).mean()
        second_turn = np.array(last_ssim_info[1]).mean()
        # print(first_turn, second_turn)
        if (first_turn > second_turn):
            return 1
        else:
            return 2
