import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from PyQt5 import QtGui, QtCore
import time
from get_click_xyz import Get3Dcoordinates
import math

from hsr_yolov3.yolo_detector import yolo_detector

class rgbOut(QtCore.QObject):
    """Get video data from the HSR"""
    changePixmap = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self, network):
        super(rgbOut, self).__init__()
        topic_name = '/hsrb/head_rgbd_sensor/rgb/image_raw'
        #topic_name = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        self._bridge = CvBridge()
        self._input_image = None
        self.zoom_state = 1
        self.maxzoom = 4
        self.minzoom = 1

        self.schemaNetwork = network;

        # Initialize yolo detector
        self.detector = yolo_detector()
        self.items = {}
        
        # Subscribe color image data from HSR
        self._image_sub = rospy.Subscriber(topic_name, Image, self._color_image_cb)
        # Wait until connection
        rospy.wait_for_message(topic_name, Image, timeout=5.0)

    def _color_image_cb(self, data):

        try:
            self._input_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            rgbImage = cv2.cvtColor(self._input_image, cv2.COLOR_BGR2RGB)

            h = 480
            w = 640

            k = 2**(self.zoom_state - 1)
            wmin = ((k-1)*w)/(2*k)
            wmax = wmin + (w/k)

            hmin = ((k-1)*h)/(2*k)
            hmax = hmin + (h/k)

            cropped_img = rgbImage[hmin:hmax, wmin:wmax]
            dst = cv2.resize(cropped_img, None, fx=k, fy=k)

            self.curr_image = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)

            #Converts from cv2 to Qt object, Intercept and apply object labeling here
            boxes,class_names,dst = self.detector.detect(dst)
            for b in boxes:
                if b[5] > .75:
                    self.items[class_names[b[6]]] = b[0],b[1],time.time()

            time_now = time.time()
            for item in self.items.keys():
                if abs(time_now-self.items[item][2]) > 60:
                    del self.items[item]

            cropped_img = QtGui.QImage(dst.data, dst.shape[1], dst.shape[0],QtGui.QImage.Format_RGB888)
            self.changePixmap.emit(cropped_img)
            self.schemaNetwork.update_context_items(self.items.keys())

        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def process_grabbable_objects(self):
        for item in self.items.keys():
                if item in self.schemaNetwork.possible_grabbable_items and item in self.items and (time.time()-self.items[item][2]) < .1:
                    print "finding " + item
                    finder3D = Get3Dcoordinates(int(round(640*self.items[item][0])),int(round(480*self.items[item][1])))
                    while True:
                        print "processing"
                        if finder3D.found_3d:
                            break
                    target_x = finder3D.map_point.point.x
                    target_y = finder3D.map_point.point.y
                    target_z = finder3D.map_point.point.z

                    del finder3D

                    if not math.isnan(target_x) and not math.isnan(target_y):
                        self.schemaNetwork.update_current_schema(item,[target_x,target_y])
                    else:
                        print "not detected"
    def save_image(self):
        if self.curr_image is not None:
            cv2.imwrite( "./trainingset/"+str(time.time())+".jpg", self.curr_image);

    def get_item_coordinates(self,item_name):
        if item_name in self.items:
            return self._input_image.shape,(self.items[item_name][0],self.items[item_name][1]),self.items[item_name][2]
        else:
            return None,None,None
    def clicked_zoom_in(self):
        if self.zoom_state < self.maxzoom:
            self.zoom_state += 1

    def clicked_zoom_out(self):
        if self.zoom_state > self.minzoom:
            self.zoom_state = self.zoom_state - 1
