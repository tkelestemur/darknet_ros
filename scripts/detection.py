#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from darkflow_msgs.msg import Detection
from darkflow_srvs.srv import GetDetections, GetDetectionsResponse
from std_srvs.srv import Empty
from darkflow.darkflow.net.build import TFNet

import cv2
import numpy as np

options1 = {"model": "cfg/yolov2.cfg", 
           "load": "bin/yolov2.weights",
           "threshold": 0.4, 
           "labels": "cfg/coco.names"}

class DarkflowROS():

    def __init__(self,options):

        rospy.init_node('darkflow_ros')

        rgb_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_cb)
        # self.detection_srv = rospy.Service('/darkflow_ros/detect', Empty, self.detection_service)
        self.detection_srv = rospy.Service('/darkflow_ros/detect', GetDetections, self.detection_service)

        self.cv_bridge = CvBridge()
        self.tfnet = TFNet(options)

        rospy.sleep(1) # settle down

    def rgb_cb(self, msg):

        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.first_image = True

        except CvBridgeError, e:
            a=e

    def detection_service(self, req):
        rospy.loginfo('it is working fine')
        imgcv = self.frame
        result = tfnet.return_predict(imgcv) # Dict. of detections
        imgcv_labels = imgcv.copy() # Image to be annotated with boxes and labels
        for box in result:
            imgcv_labels = tfnet.drawdict(imgcv_labels,box)
        detections = []
        for box in result:
            msg = Detection()
            msg.label = box['label']
            msg.bbox = [box['topleft']['x'],
                        box['topleft']['y'],
                        box['bottomright']['x'],
                        box['bottomright']['y']]
            msg.accuracy = box['confidence']
            detections.append(msg)
        return GetDetectionsResponse(detections)
        # get self.frame and detect bbox using darkflow

if __name__ == '__main__':

    detector = DarkflowROS(options1)
    rospy.spin()
