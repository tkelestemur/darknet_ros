#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
# from ..darkflow_msgs.msg import Detection
# from ..darkflow_srvs.srv import GetDetections, GetDetectionsResponse
from std_srvs.srv import Empty
from darkflow.net.build import TFNet

import cv2
import numpy as np
import darknet

yolov2_network = darknet.load_net("cfg/river2.cfg", "weights/river2.backup", 0)
yolov2_meta = darknet.load_meta("cfg/river.data")
groceries_network = darknet.load_net("cfg/yolov2.cfg", "weights/yolov2.weights", 0)
groceries_meta = darknet.load_meta("cfg/coco.data")

class DarkflowROS():

    def __init__(self,options):

        rospy.init_node('darkflow_ros')

        rgb_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_cb)
        self.detection_srv = rospy.Service('/darkflow_ros/detect', Empty, self.detection_service)
        # self.detection_srv = rospy.Service('/darkflow_ros/detect', GetDetections, self.detection_service)

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
        img = self.frame
        results = darknet.detect(groceries_network, groceries_meta, img, thresh=0.35)

        for box in results:
            label = box[0]
            w = int(box[2][2])
            h = int(box[2][3])
            cx = int(box[2][0])
            cy = int(box[2][1])
            bx = int(cx-(w/2))
            by = int(cy-(h/2))
            conf = round(r[k][1],2)
            img = cv2.rectangle(img,(bx,by),
                                (bx+w,by+h),
                                (0,0,255),
                                2)
            thick = int((h + w) // 300)*3
            cv2.putText(img, label, (bx, by - 12),
                0, 1e-3 * img.shape[0],(0,0,255),thick//3)
            cv2.putText(img, str(conf), (bx, by - 36),
                0, 1e-3 * img.shape[0],(0,0,255),thick//3)
        print results
        cv2.imshow("labelled_img",img)
        cv2.waitKey()
        cv2.destroyAllWindows()
        # return GetDetectionsResponse(detections)

if __name__ == '__main__':

    detector = DarkflowROS(options1)
    rospy.spin()