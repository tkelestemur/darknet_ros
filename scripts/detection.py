#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
# from detection_msgs.msg import Detection
# from detection_srvs.srv import GetDetections, GetDetectionsResponse
from std_srvs.srv import Empty

import cv2
import os
import numpy as np
import darknet_cv2 as darknet

# File Organization
# pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
pkg_dir = "/home/nyokoyama/ros_ws/src/darknet_ros"
cfg_dir = pkg_dir+"/cfg/"
weights_dir = pkg_dir+"/weights/"
# Load models
groceries_network = darknet.load_net(cfg_dir+"river2.cfg", weights_dir+"river2.backup", 0)
groceries_meta = darknet.load_meta(cfg_dir+"river.data")
yolov2_network = darknet.load_net(cfg_dir+"yolov2.cfg", weights_dir+"yolov2.weights", 0)
yolov2_meta = darknet.load_meta(cfg_dir+"coco.data")

class DarknetROS():

    def __init__(self):

        rospy.init_node('darknet_ros')

        # rgb_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        rgb_topic = "/hsrb/head_r_stereo_camera/image_raw"
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_cb)
        self.detection_srv = rospy.Service('/darknet_ros/detect', Empty, self.detection_service)
        # self.detection_srv = rospy.Service('/darknet_ros/detect', GetDetections, self.detection_service)

        self.cv_bridge = CvBridge()

        rospy.sleep(1) # settle down

    def rgb_cb(self, msg):

        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.first_image = True

        except CvBridgeError, e:
            a=e

    def detection_service(self, req):
        img = self.frame
        # cv2.imshow("input",img)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        # results = darknet.detect(yolov2_network, yolov2_meta, img, thresh=0.35)
        results = darknet.detect(groceries_network, groceries_meta, img, thresh=0.35)
        print results
        for box in results:
            label = box[0]
            conf = round(box[1],2)
            w = int(box[2][2])
            h = int(box[2][3])
            cx = int(box[2][0])
            cy = int(box[2][1])
            bx = int(cx-(w/2))
            by = int(cy-(h/2))
            img = cv2.rectangle(img,(bx,by),
                                (bx+w,by+h),
                                (0,0,255),
                                2)
            cv2.putText(img, label, (bx, by - 12),
                0, 1e-3 * img.shape[0],(0,0,255),2)
            cv2.putText(img, str(conf), (bx, by - 36),
                0, 1e-3 * img.shape[0],(0,0,255),2)
        cv2.imshow("labelled_img",img)
        cv2.waitKey()
        cv2.destroyAllWindows()
        # return GetDetectionsResponse(detections)
        return True

if __name__ == '__main__':
    dark = DarknetROS()
    dark.detection_service("")
    rospy.spin()