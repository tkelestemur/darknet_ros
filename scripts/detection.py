#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Empty

import cv2
import numpy as np


class DarkflowROS():

    def __init__(self):

        rospy.init_node('darkflow_ros')

        rgb_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_cb)
        self.detection_srv = rospy.Service('/darkflow_ros/detect', Empty, self.detection_service)

        self.cv_bridge = CvBridge()

        rospy.sleep(1) # settle down

    def rgb_cb(self, msg):

        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.first_image = True

        except CvBridgeError, e:
            a=e

    def detection_service(self, req):
        rospy.loginfo('it is working fine')
        # get self.frame and detect bbox using darkflow


if __name__ == '__main__':

    detector = DarkflowROS()
    rospy.spin()
