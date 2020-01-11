import sys
import time
import cv2
from PIL import Image, ImageDraw
from models.tiny_yolo import TinyYoloNet
from utils import *
from darknet import Darknet

if __name__ == '__main__':
    # Set up cv detector
    m = Darknet('cfg/yolov3.cfg')
    m.print_network()
    m.load_weights('yolov3.weights')
    namesfile = 'data/coco.names'
    
    use_cuda = 1
    if use_cuda:
        m.cuda()

    # Set up video capture
    cap = cv2.VideoCapture(0)
    assert cap.isOpened(), 'Cannot capture source'
    
    # Process frames
    frames = 0
    start = time.time()    
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            # Detect
            sized = cv2.resize(frame, (m.width, m.height))
            sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)

            startdetect = time.time()
            boxes = do_detect(m, sized, 0.5, 0.4, use_cuda)
            finish = time.time()

            print('Predicted in %f seconds.' % (finish-startdetect))
            class_names = load_class_names(namesfile)
            result = plot_boxes_cv2(frame, boxes, class_names=class_names)

            cv2.imshow("frame", result)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break
            frames += 1
            print("FPS of the video is {:5.2f}".format( frames / (time.time() - start)))
        else:
            break