#! /usr/bin/env python
from __future__ import print_function
import sys
souce_path = "./src/gem_vision/camera_vision/scripts/"
# souce_path = "./src/vehicle_drivers/gem_vision/gem_vision/camera_vision/scripts/"
sys.path.append(souce_path)
from camera_utils import *
sys.path.append(souce_path + "Detector/")
from yolo_detect_image import yolo_detect_image
sys.path.append(souce_path + "lane_detect/")
from lane_detector import lane_detector

object_detection = True
lane_detection = True

class ImageConverter:
    def __init__(self):
        self.node_name = "gem_vision"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.last_state_info = [[], []]
        self.last_ssim_info = [deque(), deque()]
        self.frame_counter = 0
        # Subscribe camera rgb and depth information
        depth_img_topic = rospy.get_param('depth_info_topic','/zed2/zed_node/depth/depth_registered')
        self.depth_img_sub = message_filters.Subscriber(depth_img_topic,Image)
        self.subcriber_rgb = message_filters.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image)
        self.subcriber_rgb_camera = message_filters.Subscriber('/zed2/zed_node/rgb_raw/camera_info', CameraInfo)
        sync = message_filters.ApproximateTimeSynchronizer([self.subcriber_rgb, self.depth_img_sub, self.subcriber_rgb_camera], 10, 1)
        sync.registerCallback(self.multi_callback)
        # Publish Boudingbox information of objects
        self.image_pub = rospy.Publisher("/object_detection", Detected_msg, queue_size=1)
        self.image_pub_rviz = rospy.Publisher("/object_detection_person", Image, queue_size=1)

    def multi_callback(self, rgb, depth, camera_info):
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            depth_frame = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        out_frame = rgb_frame
        detectBox = Detected_msg()
        # ----------------- Imaging processing code starts here ----------------\
        # # Object Detection
        if object_detection == True:
            detected_list, out_frame = yolo_detect_image(out_frame, souce_path)
            object_camera_coordinate_list, out_frame = calculate_object_distance(detected_list, depth_frame, camera_info, out_frame)
        # Add information you want to publish

        # Lane Detection 
        if lane_detection == True:
            self.frame_counter = self.frame_counter + 1
            middle_point, out_frame, curren_state_info, current_ssim_info, signal, angle, segmented_image= lane_detector(out_frame, self.last_state_info, self.last_ssim_info, self.frame_counter, souce_path)
            self.last_state_info = curren_state_info
            self.current_ssim_info = current_ssim_info
            lane_camera_coordinate_list, bbx_frame  = calculate_lane_distance(middle_point, depth_frame, camera_info, out_frame)
        # Add information you want to publish

        cv2imshow(self, out_frame, "out_frame", 1)
        # ----------------------------------------------------------------------
        self.image_pub.publish(detectBox)
        cv_img = CvBridge().cv2_to_imgmsg(out_frame, "bgr8")
        self.image_pub_rviz.publish(cv_img)

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

    