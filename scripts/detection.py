#!/usr/bin/env python

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from std_srvs.srv import Empty
from darknet_ros.msg import Detection
from darknet_ros.srv import GetDetections, GetDetectionsResponse

import cv2
import os
import numpy as np
import darknet_cv2 as darknet

class DarknetROS():

    def __init__(self):

        rospy.init_node('darknet_ros')
        rospack = rospkg.RosPack()
        pkg_dir = rospack.get_path('darknet_ros')
        cfg_dir = pkg_dir + "/cfg/groceries_v2/"
        weights_dir = pkg_dir + "/weights/groceries_v2/"
        # weights_name = "river2_13hrs.backup"
        weights_name = "river_less_noise_100000_unfrozen_49hrs.backup"

        self.groceries_network = darknet.load_net(cfg_dir + "river2.cfg", weights_dir + weights_name, 0)
        data_file = darknet.generate_data_file(cfg_dir,"river2.cfg","river2.names")
        self.groceries_meta = darknet.load_meta(data_file)

        rgb_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        camera_info_topic = "/hsrb/head_rgbd_sensor/depth_registered/camera_info"
        detection_image_topic = "/darknet_ros/detection_image"

        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_cb)
        self.rgb_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.rgb_info_cb)
        self.detection_image_pub = rospy.Publisher(detection_image_topic, Image, queue_size=10)
        self.detection_srv = rospy.Service('/frasier/darknet_ros/detect', GetDetections, self.detection_service)

        self.cam_model = PinholeCameraModel()
        self.cv_bridge = CvBridge()

        self.rate = 10
        rospy.sleep(1) # settle down


    def rgb_info_cb(self, msg):
        # self.cam_info = msg
        self.cam_model.fromCameraInfo(msg)
        self.rgb_info_sub.unregister()

    def rgb_cb(self, msg):

        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.first_image = True

        except CvBridgeError, e:
            a=e

    def detection_service(self, req):

        img = self.frame

        results = darknet.detect(self.groceries_network, self.groceries_meta, img, thresh=0.40)
        self._draw_bbox(img, results)
        detections = []
        for box in results:
            label = box[0]
            conf = round(box[1],2)
            w = int(box[2][2])
            h = int(box[2][3])
            cx = int(box[2][0])
            cy = int(box[2][1])
            if (cx-(w/2)) > 0:
                left = cx - (w/2)
            else:
                left = 0
            if (cy-(h/2)) > 0:
                top = cy - (h/2)
            else:
                top = 0
            if left+w < np.size(img, 1):
                right = left + w
            else:
                right = np.size(img, 1)
            if top+h < np.size(img, 0):
                bottom = top + h
            else:
                bottom = np.size(img, 0)

            detection = Detection()
            detection.header.stamp = rospy.Time.now()
            detection.label = label
            detection.bbox = [left, top, right, bottom]
            detection.center = [cx, cy]
            detection.accuracy = box[1]
            msg = self.cv_bridge.cv2_to_imgmsg(img)
            self.detection_image_pub.publish(msg)
            detections.append(detection)


        return GetDetectionsResponse(detections)

    def continuous_detection_service(self, req):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            img = self.frame
            results = darknet.detect(self.groceries_network, self.groceries_meta, img, thresh=0.35)
            self._draw_bbox(img, results)
            try:
                msg = self.cv_bridge.cv2_to_imgmsg(img)
            except CvBridgeError, e:
                a=e
            self.detection_image_pub.publish(msg)
            rate.sleep()


    def _draw_bbox(self, img, results):
        for box in results:
            label = box[0]
            conf = round(box[1],2)
            w = int(box[2][2])
            h = int(box[2][3])
            cx = int(box[2][0])
            cy = int(box[2][1])
            if (cx-(w/2)) > 0:
                left = cx - (w/2)
            else:
                left = 0
            if (cy-(h/2)) > 0:
                top = cy - (h/2)
            else:
                top = 0
            right = left + w
            bottom = top + h

            img = cv2.rectangle(img, (left, top), (right, bottom), (0,0,255), 2)
            cv2.putText(img, label, (left, top - 12), 0, 1e-3 * img.shape[0],(0,0,255),2)
            cv2.putText(img, str(conf), (left, top - 36), 0, 1e-3 * img.shape[0],(0,0,255),2)
            cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)



if __name__ == '__main__':
    dark = DarknetROS()
    # dark.detect()
    rospy.spin()
