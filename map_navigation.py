from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QPixmap
from PyQt5.QtCore import Qt, QPoint, QTimer
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
import tf.transformations
import math
import arm_control, gripper_control

# Imports for testing actionlib goal movements
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetMap

import re
import numpy as np
import sys

class AutoNavigate(QWidget):

    def __init__(self):
        super(AutoNavigate, self).__init__()
        self.map_resolution = .05  # meters per pixel
        #self.map_dimensions = (200, 200)  # columns, rows
        self.map_dimensions = (234, 314)
        self.center = self.map_dimensions[0] / 2, self.map_dimensions[1] / 2

        self.canvas_pose = (10, 10)  # variable to store robot's coordinates on the pixmap
        self.global_pose = (0, 0)  # variable to store robot's coordinates on the pixmap
        self.goal_canvas_pose = (0, 0)  # variable to store goal coordinates on the pixmap
        self.traveling = False
        #self.scale = 2.5  # how big you want the map to show up, just class
        self.scale = 1.6
        self.mousePressEvent = self.get_pos
        self.armMove = arm_control.ArmControl()
        self.gripper_move = gripper_control.GripperControl()

        #self.pgm = self.read_pgm("class-map-v2/map.pgm", byteorder='<')
        self.pgm = self.read_pgm("lab-map/map.pgm", byteorder='<')

        self.canvas_gridpts = []

        self.temp_points = []

        self.global_pose_sub = rospy.Subscriber('/global_pose', PoseStamped, self.receive_state)
        rospy.wait_for_message('/global_pose', PoseStamped, timeout=5.0)

        self.recording = False
        self.recordfile = open("trajectory.txt","w+")

        '''
        self.goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
        # wait to establish connection between the navigation interface
        while self.goal_pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        '''

        # initialize action client
        self.cli_nav = actionlib.SimpleActionClient(
            '/move_base/move/',
            MoveBaseAction)
        # wait for the action server to establish connection
        self.cli_nav.wait_for_server()

        self.initUI()
        self.timer = QTimer()
        self.timer.timeout.connect(self.repaint)
        self.timer.start(500)

    def initUI(self):
        self.setGeometry(10, 10, self.map_dimensions[0] * self.scale, self.map_dimensions[1] * self.scale)
        # We can add a Qtcore.timer here to call update() in every x ms
        self.show()

    def updateGridpts(self,global_gridpts):
        for gp in global_gridpts:
            self.canvas_gridpts.append(self.convert_global_to_canvas(gp))

    def paintEvent(self, event):
        #pixmap = QPixmap("class-map-v2/map_cropped_tagged3.pgm")  # cropped pgm must have the same center as original
        pixmap = QPixmap("lab-map/map_cropped_tagged.pgm")
        size = pixmap.size()
        scaledPix = pixmap.scaled(size*self.scale, Qt.KeepAspectRatio)

        painter = QPainter(self)
        painter.drawPixmap(0,0, scaledPix, 0,0,self.map_dimensions[0] * self.scale,self.map_dimensions[1] * self.scale)

        penGrid = QPen(Qt.blue,3)
        painter.setPen(penGrid)
        for gp in self.canvas_gridpts:
            painter.drawEllipse(QPoint(gp[0], gp[1]), .5 * self.scale, .5 * self.scale)

        pen = QPen(Qt.red, 3)
        painter.setPen(pen)
        painter.drawEllipse(QPoint(self.canvas_pose[0], self.canvas_pose[1]), 2 * self.scale, 2 * self.scale)

        penTemp = QPen(Qt.black, 3)
        painter.setPen(penTemp)
        for tp in self.temp_points:
            painter.drawEllipse(QPoint(tp[0], tp[1]), .2 * self.scale, .2 * self.scale)

        if self.traveling:
            pen2 = QPen(Qt.green, 3)
            painter.setPen(pen2)
            painter.drawEllipse(QPoint(self.goal_canvas_pose[0], self.goal_canvas_pose[1]), 2 * self.scale,
                                2 * self.scale)

        if self.recording:
            self.recordfile.write(str(self.global_pose[0])+','+str(self.global_pose[1])+'\n')

    def receive_state(self, data):
        self.global_pose = (data.pose.position.x, data.pose.position.y)
        self.canvas_pose = self.convert_global_to_canvas(self.global_pose)
        if self.traveling:
            if math.sqrt(math.pow((self.canvas_pose[0]-self.goal_canvas_pose[0]), 2)+math.pow((self.canvas_pose[1]-self.goal_canvas_pose[1]), 2))/self.scale < 2:
                self.traveling = False
        #self.repaint()
        #self.update()

    def convert_global_to_canvas(self, global_coord):
        # convert global_coords to canvas pixels, still cartesian
        pixels = global_coord[0]/self.map_resolution, global_coord[1]/self.map_resolution
        # convert cartesian canvas pixels to c,r pixels
        crpixels = self.center[0]+pixels[0], self.center[1]-pixels[1]
        # scale up c,r pixels to fit screen and return
        return crpixels[0]*self.scale, crpixels[1]*self.scale

    def convert_canvas_to_global(self, canvas_coord):
        # unscale pixels to c,r pixels
        crpixels = canvas_coord[0]/self.scale, canvas_coord[1]/self.scale
        # convert c,r pixels to cartesian canvas pixels
        pixels = crpixels[0]-self.center[0], self.center[1]-crpixels[1]
        # convert cartesian canvas pixels to global coords and return
        return pixels[0]*self.map_resolution, pixels[1]*self.map_resolution

    def get_pos(self, event):
        self.goal_canvas_pose = (event.pos().x(), event.pos().y())
        goal_pose = self.convert_canvas_to_global((event.pos().x(), event.pos().y()))
        print('Going to: ' + str(goal_pose))
        self.traveling = True
        self.repaint()

        '''
        # fill ROS message
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position = Point(goal_pose[0], goal_pose[1], 0)
        # goal yaw
        quat = tf.transformations.quaternion_from_euler(0, 0, math.atan2(goal_pose[1] - self.global_pose[1], goal_pose[0] - self.global_pose[0]))
        goal.pose.orientation = Quaternion(*quat)
        '''

        self.go_to_mapXY(goal_pose[0], 0.0, goal_pose[1], 0.0, math.atan2(goal_pose[1] - self.global_pose[1], goal_pose[0] - self.global_pose[0]))

        # publish ROS message
        #self.goal_pub.publish(goal)

        self.armMove.clicked_arm_neutral()
        self.gripper_move.clicked_gripper_close()

    '''
    def go_to_mapXY(self, target_x, x_offset, target_y, y_offset):
        x = target_x + x_offset
        y = target_y + y_offset
        print('Going to: ' + str(x) + ' ' + str(y))
        self.traveling = True
        self.repaint()

        # fill ROS message
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position = Point(x, y, 0)
        # goal yaw
        quat = tf.transformations.quaternion_from_euler(0, 0, math.atan2(target_y - y, target_x - x))
        goal.pose.orientation = Quaternion(*quat)

        # publish ROS message
        self.goal_pub.publish(goal)
        #while math.sqrt(math.pow((x - self.global_pose[0]), 2)+math.pow((y - self.global_pose[1]), 2)) > 0.1:
        #    rospy.sleep(0.2)
    '''
    
    def go_to_mapXY(self, target_x, x_offset, target_y, y_offset, yaw_offset, yaw=None):
        x = target_x + x_offset
        y = target_y + y_offset
        print('Going to: ' + str(x) + ' ' + str(y))
        self.traveling = True
        self.repaint()

        # initialize action client
        #cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        # wait for the action server to establish connection
        #cli.wait_for_server()
        # fill ROS message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x, y, 0)
        # goal yaw
        if yaw is None:
            quat = tf.transformations.quaternion_from_euler(0, 0, math.atan2(target_y - y, target_x - x)+yaw_offset)
        else:
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw+yaw_offset)
        pose.pose.orientation = Quaternion(*quat)
        goal = MoveBaseGoal()
        goal.target_pose = pose
        # send message to the action server
        self.cli_nav.send_goal(goal)
        # wait for the action server to complete the order
        self.cli_nav.wait_for_result(rospy.Duration(secs=20))
        # print result of navigation
        action_state = self.cli_nav.get_state()
        if action_state == GoalStatus.SUCCEEDED:
            print 'Navigation Succeeded.'
        else:
            print 'Navigation Failed.'
            raise Exception('Navigation Failed')

        #while math.sqrt(math.pow((x - self.global_pose[0]), 2)+math.pow((y - self.global_pose[1]), 2)) > 0.1:
        #    rospy.sleep(0.2)

    def calc_best_destination(self,x,y,r,n):
        angles = np.arange(0,2*np.pi,2*np.pi/n)
        points = []
        for a in angles:
            points.append([x+r*np.cos(a),y+r*np.sin(a)])
        maxDist = 0
        bestPoint = points[0]
        self.temp_points = []
        for p in points:
            dist = self.obstacle_dist(p[0],p[1])
            print dist
            #self.temp_points.append(self.convert_global_to_canvas(p))
            if dist > maxDist:
                self.temp_points = [self.convert_global_to_canvas(p)]
                maxDist = dist
                bestPoint = p
        if self.is_occupied(bestPoint[0],bestPoint[1]):
            return None
        else:
            return bestPoint
        return None

    def obstacle_dist(self,x,y):
        x_map = x/self.map_resolution
        y_map = y/self.map_resolution
        r_map = int(2048/2.0-y_map)
        c_map = int(2048/2.0+x_map)
        # BFS
        candidates = [[r_map,c_map]]
        visited = []
        while len(candidates) > 0:
            candidate = candidates.pop(0)
            if self.pgm[candidate[0],candidate[1]] == 205 or self.pgm[candidate[0],candidate[1]] == 0:
                return self.map_resolution * math.sqrt(math.pow(r_map-candidate[0],2)+math.pow(c_map-candidate[1],2))
            for r in (candidate[0]-1,candidate[0],candidate[0]+1):
                if r > -1 and r < self.pgm.shape[0]:
                    for c in (candidate[1]-1,candidate[1],candidate[1]+1):
                        if c > -1 and c < self.pgm.shape[1]:
                            if [r,c] not in visited:
                                candidates.append([r,c])
                                visited.append([r,c])
        return sys.maxint

    def is_occupied(self,x,y):
        #from matplotlib import pyplot
        #pyplot.imshow(self.pgm, pyplot.cm.gray)
        #pyplot.show()
        '''
        print "a1"
        showmap = rospy.ServiceProxy('get_static_obstacle_map', GetMap)
        print "a2"
        print showmap
        print showmap()
        print showmap().map
        occupancy = showmap().map
        
        data = occupancy.data
        info = occupancy.info
        '''
        x_map = x/self.map_resolution
        y_map = y/self.map_resolution
        r_map = int(2048/2.0-y_map)
        c_map = int(2048/2.0+x_map)

        if self.pgm[r_map,c_map]==205 or self.pgm[r_map,c_map]==0:
            print "True"
            return True
        else:
            print "False"
            return False

    def read_pgm(self,filename, byteorder='>'):
        """Return image data from a raw PGM file as numpy array.

        Format specification: http://netpbm.sourceforge.net/doc/pgm.html

        """
        with open(filename, 'rb') as f:
            buffer = f.read()
        try:
            header, width, height, maxval = re.search(
                b"(^P5\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
        except AttributeError:
            raise ValueError("Not a raw PGM file: '%s'" % filename)
        return np.frombuffer(buffer,
                                dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                                count=int(width)*int(height),
                                offset=len(header)
                                ).reshape((int(height), int(width)))