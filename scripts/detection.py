#!/usr/bin/env python

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from darknet_ros.msg import Detection
from darknet_ros.srv import GetDetections, GetDetectionsResponse
from std_srvs.srv import Empty

import cv2
import os
import numpy as np
import darknet_cv2 as darknet

# File Organization
# pkg_dir = "/home/nyokoyama/ros_ws/src/darknet_ros"
# cfg_dir = pkg_dir+"/cfg/"
# weights_dir = pkg_dir+"/weights/"
# Load models
# groceries_network = darknet.load_net(cfg_dir+"river2.cfg", weights_dir+"river2.backup", 0)
# groceries_meta = darknet.load_meta(cfg_dir+"river.data")
# yolov2_network = darknet.load_net(cfg_dir+"yolov2.cfg", weights_dir+"yolov2.weights", 0)
# yolov2_meta = darknet.load_meta(cfg_dir+"coco.data")

class DarknetROS():

    def __init__(self):

        rospy.init_node('darknet_ros')
        self.setupParams()
        rgb_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        # rgb_topic = "/hsrb/head_rgbd_sensor/rgb/image_raw"
        # rgb_topic = "/hsrb/head_r_stereo_camera/image_raw"
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_cb)
        # self.detection_srv = rospy.Service('/darknet_ros/detect', Empty, self.detection_service)
        self.detection_srv = rospy.Service('/darknet_ros/detect', GetDetections, self.detection_service)

        self.cv_bridge = CvBridge()

        rospack = rospkg.RosPack()
        pkg_dir = rospack.get_path('darknet_ros')
        cfg_dir = pkg_dir + "/cfg/"
        weights_dir = pkg_dir + "/weights/"

        self.groceries_network = darknet.load_net(cfg_dir+"river2.cfg", weights_dir+"river2.backup", 0)
        self.groceries_meta = darknet.load_meta(cfg_dir+"river.data")
        rospy.sleep(1) # settle down

    def rgb_cb(self, msg):

        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.first_image = True

        except CvBridgeError, e:
            a=e

    def detection_service(self, req):
        img = self.frame
        # results = darknet.detect(yolov2_network, yolov2_meta, img, thresh=0.35)
        results = darknet.detect(self.groceries_network, self.groceries_meta, img, thresh=0.35)
        print results
        detections = []
        for box in results:
            label = box[0]
            conf = round(box[1],2)
            w = int(box[2][2])
            h = int(box[2][3])
            cx = int(box[2][0])
            cy = int(box[2][1])
            bx = int(cx-(w/2))
            by = int(cy-(h/2))
            detection = Detection()
            detection.label = label
            detection.bbox = [bx,by,bx+w,by+h]
            detection.accuracy = box[1]
            img = cv2.rectangle(img,(bx,by),
                                (bx+w,by+h),
                                (0,0,255),
                                2)
            cv2.putText(img, label, (bx, by - 12),
                0, 1e-3 * img.shape[0],(0,0,255),2)
            cv2.putText(img, str(conf), (bx, by - 36),
                0, 1e-3 * img.shape[0],(0,0,255),2)
            detections.append([detection])
        cv2.imshow("labelled_img",img)
        cv2.waitKey()
        cv2.destroyAllWindows()
        print detections
        return GetDetectionsResponse(detections)

    def setupParams(self):
        rospy.set_param("/hsrb/head_rgbd_sensor/property/auto_exposure/on_off",True)
        rospy.set_param("/hsrb/head_rgbd_sensor/property/auto_exposure/auto_manual_mode",False)
        rospy.set_param("/hsrb/head_rgbd_sensor/property/auto_exposure/abs_value",9)
        rospy.set_param("/hsrb/head_rgbd_sensor/property/auto_exposure/one_push",False)

        # rospy.set_param("/hsrb/stereo_camera/property/auto_exposure/on_off",True)
        # rospy.set_param("/hsrb/stereo_camera/property/auto_exposure/auto_manual_mode",True)
        # rospy.set_param("/hsrb/stereo_camera/property/auto_exposure/abs_value",)
        # rospy.set_param("/hsrb/stereo_camera/property/auto_exposure/one_push",False)

        # rospy.set_param("/hsrb/stereo_camera/property/brightness/on_off",True)
        # rospy.set_param("/hsrb/stereo_camera/property/brightness/auto_manual_mode",True)
        # rospy.set_param("/hsrb/stereo_camera/property/brightness/abs_value",1.0)
        # rospy.set_param("/hsrb/stereo_camera/property/brightness/one_push",False)

        # rospy.set_param("/hsrb/stereo_camera/property/gain/on_off",True)
        # rospy.set_param("/hsrb/stereo_camera/property/gain/auto_manual_mode",True)
        # rospy.set_param("/hsrb/stereo_camera/property/gain/one_push",True)
        # rospy.set_param("/hsrb/stereo_camera/property/brightness/abs_value",9990.0)

        # rospy.set_param("/hsrb/stereo_camera/property/shutter/on_off",True)
        # rospy.set_param("/hsrb/stereo_camera/property/shutter/auto_manual_mode",True)
        # rospy.set_param("/hsrb/stereo_camera/property/shutter/one_push",False)

        # rospy.set_param("/hsrb/stereo_camera/property/white_balance/on_off",True)
        # rospy.set_param("/hsrb/stereo_camera/property/white_balance/auto_manual_mode",True)
        # rospy.set_param("/hsrb/stereo_camera/property/white_balance/one_push",False)

if __name__ == '__main__':
    dark = DarknetROS()
    dark.detection_service("")
    rospy.spin()

        # print rospy.get_param("/hsrb/head_rgbd_sensor/rgb/image_rect_color/compressed/format")
        # print rospy.get_param("/hsrb/head_rgbd_sensor/rgb/image_rect_color/compressed/jpeg_quality")
        # print rospy.get_param("/hsrb/head_rgbd_sensor/rgb/image_rect_color/compressed/png_level")

        # print rospy.get_param("/hsrb/stereo_camera/property/auto_exposure/abs_value")
        # print rospy.get_param("/hsrb/stereo_camera/property/auto_exposure/auto_manual_mode")
        # print rospy.get_param("/hsrb/stereo_camera/property/auto_exposure/on_off")
        # print rospy.get_param("/hsrb/stereo_camera/property/auto_exposure/one_push")

        # print rospy.get_param("/hsrb/stereo_camera/property/brightness/abs_value")
        # print rospy.get_param("/hsrb/stereo_camera/property/brightness/auto_manual_mode")
        # print rospy.get_param("/hsrb/stereo_camera/property/brightness/on_off")
        # print rospy.get_param("/hsrb/stereo_camera/property/brightness/one_push")

        # print rospy.get_param("/hsrb/stereo_camera/property/gain/abs_value")
        # print rospy.get_param("/hsrb/stereo_camera/property/gain/auto_manual_mode")
        # print rospy.get_param("/hsrb/stereo_camera/property/gain/on_off")
        # print rospy.get_param("/hsrb/stereo_camera/property/gain/one_push")

        # print rospy.get_param("/hsrb/stereo_camera/property/shutter/abs_value")
        # print rospy.get_param("/hsrb/stereo_camera/property/shutter/auto_manual_mode")
        # print rospy.get_param("/hsrb/stereo_camera/property/shutter/on_off")
        # print rospy.get_param("/hsrb/stereo_camera/property/shutter/one_push")

        # print rospy.get_param("/hsrb/stereo_camera/property/white_balance/auto_manual_mode")
        # print rospy.get_param("/hsrb/stereo_camera/property/white_balance/on_off")
        # print rospy.get_param("/hsrb/stereo_camera/property/white_balance/one_push")
        # print rospy.get_param("/hsrb/stereo_camera/property/white_balance/value_a")
        # print rospy.get_param("/hsrb/stereo_camera/property/white_balance/value_b")
        # return GetDetectionsResponse(detection)
