import sys
import time
import cv2
from PIL import Image, ImageDraw
from hsr_yolov3.tiny_yolo import TinyYoloNet
from utils import *
from darknet import Darknet

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class yolo_detector(object):
    """Get video data from the HSR"""
    def __init__(self):
        # Set up cv detector
        import os
        print os.getcwd()
        self._m = Darknet('hsr_yolov3/cfg/yolov3.cfg')
        self._m.print_network()
        self._m.load_weights('hsr_yolov3/yolov3.weights')
        self._namesfile = 'hsr_yolov3/data/coco.names'

        self._use_cuda = 1
        if self._use_cuda:
            self._m.cuda()

        # Process frames
        self._frames = 0
        self._start = time.time()

        topic_name = '/hsrb/head_rgbd_sensor/rgb/image_raw'
        self._bridge = CvBridge()
        self._input_image = None

    def detect(self, data):
        try:
            #self._input_image = self._bridge.imgmsg_to_cv2(data, "bgr8")

            # Detect
            #sized = cv2.resize(self._input_image, (self._m.width, self._m.height))
            #sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)

            startdetect = time.time()
            #boxes = do_detect(self._m, sized, 0.5, 0.4, self._use_cuda)
            boxes = do_detect(self._m, data, 0.5, 0.4, self._use_cuda)
            finish = time.time()

            #print('Predicted in %f seconds.' % (finish-startdetect))
            class_names = load_class_names(self._namesfile)
            #result = plot_boxes_cv2(self._input_image, boxes, class_names=class_names)
            result = plot_boxes_cv2(data, boxes, class_names=class_names)

            #cv2.imshow("frame", result)
            #key = cv2.waitKey(1)
            self._frames += 1
            #print("FPS of the video is {:5.2f}".format( self._frames / (time.time() - self._start)))
            return boxes,class_names,result
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)
            return None