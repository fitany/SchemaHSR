import sys
from PyQt5.QtWidgets import QApplication, QDialog, QGridLayout, QGroupBox, QPushButton, QLabel, QLineEdit, QSizePolicy
from PyQt5 import QtGui, QtCore
import os
import xtion_channels, button_control, arm_control, map_navigation, gripper_control, speech_control
import rospy
import yaml
import hsrb_interface
from get_click_xyz import Get3Dcoordinates
import math
import hand_control_ui
import high_res_cam_ui
import thread
import network, network_ui
import time
import numpy as np
import copy


class Thread(QtCore.QThread):
    def run(self):
        with open("param.yaml", "r") as f:
            param = yaml.load(f)
        QtCore.QThread.sleep(param["rotate_90"]["time_to_rotate"])


class ClientUI(QDialog):

    def __init__(self):
        super(ClientUI, self).__init__()
        self.title = 'Main Interface'
        self.robot = hsrb_interface.Robot()
        #self.whole_body = self.robot.get('whole_body')
        self.omni_base = self.robot.get('omni_base')
        self.gripper = self.robot.get('gripper')

        self.left = 500
        self.top = 100
        self.width =1350
        self.height = 900
        self.active_reach_target = False
        self.active_put_down = False

        self.schemaNetwork = network.SchemaNetwork()

        self.logopath = os.path.join(os.path.curdir, 'image', 'blank.jpg')
        self.setWindowIcon(QtGui.QIcon(self.logopath))
        self.headMove = button_control.HeadControl()
        self.gripperMove = gripper_control.GripperControl()
        self.armMove = arm_control.ArmControl()
        self.cameraFeed = xtion_channels.rgbOut(self.schemaNetwork)
        self.baseMove = button_control.BaseControl()
        self.basePose = button_control.BaseTrajectoryControl()
        self.autodrive = map_navigation.AutoNavigate()
        self.speech = speech_control.SpeechControl()

        self.waypoints_classroom = [[0.0,1.5],[1.54,.64],[2.5,0.5],[1.27,-0.63],[0.0,0.0]]
        self.probabilities_classroom = [0.2,0.2,0.2,0.2,0.2]
        self.waypoints_breakroom = [[-2.21,-3.32],[-3.86,-4.75],[-2.73,-5.71],[-1.63,-5.64],[-0.57,-4.59]]
        self.probabilities_breakroom = [0.2,0.2,0.2,0.2,0.2]
        self.roam_classroom = False
        self.roam_breakroom = False

        self.autodrive.updateGridpts(self.schemaNetwork.gridpoints)

        self.timertime = 0

        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.setFixedSize(self.width, self.height)
        self.createGridLayout()
        self.show()

    def createGridLayout(self):
        self.createCameraGrid()
        self.createBaseHeadMoveGrid()
        self.createArmTrunkMoveGrid()
        self.createMapGrid()
        self.createZoomPopGrid()
        self.createClick2ReachGrid()
        gridLayout = QGridLayout()
        gridLayout.addWidget(self.cameraBox, 0, 0)
        gridLayout.addWidget(self.zoomPopGrid, 2, 0)
        gridLayout.addWidget(self.click2reachGrid, 1, 0)
        gridLayout.addWidget(self.mapBox, 0, 1)
        gridLayout.addWidget(self.baseHeadMoveGrid, 1, 1)
        gridLayout.addWidget(self.armTrunkMoveGrid, 2, 1)
        self.setLayout(gridLayout)

    @QtCore.pyqtSlot(QtGui.QImage)
    def setStream(self, image):
        self.rgblabel.setPixmap(QtGui.QPixmap.fromImage(image))

    def createBaseHeadMoveGrid(self):
        self.baseHeadMoveGrid = QGroupBox('')
        baseHeadGridLayout = QGridLayout()
        self.createBaseMoveGrid()
        self.createHeadMoveGrid()
        baseHeadGridLayout.addWidget(self.baseMoveGrid, 0, 0)
        baseHeadGridLayout.addWidget(self.headMoveGrid, 0, 1)
        self.baseHeadMoveGrid.setLayout(baseHeadGridLayout)

    def createArmTrunkMoveGrid(self):
        self.armTrunkMoveGrid = QGroupBox('')
        armTrunkGridLayout = QGridLayout()
        self.createArmMoveGrid()
        self.createTrunkMoveGrid()
        armTrunkGridLayout.addWidget(self.trunkMoveGrid, 0, 1)
        armTrunkGridLayout.addWidget(self.armMoveGrid, 0, 0)
        self.armTrunkMoveGrid.setLayout(armTrunkGridLayout)

    def createTrunkMoveGrid(self):
        self.trunkMoveGrid = QGroupBox('Adjust Height')
        trunkMoveGridLayout = QGridLayout()

        trunk_raise_button = QPushButton('Raise Height')
        trunk_raise_button.setStyleSheet("color: black; background-color: rgb(204, 153, 255)")
        trunkMoveGridLayout.addWidget(trunk_raise_button, 0, 0)
        trunk_raise_button.pressed.connect(self.armMove.clicked_raise_trunk)
        trunk_raise_button.clicked.connect(self.headMove.clicked_home)
        trunk_raise_button.setAutoRepeat(True)

        trunk_lower_button = QPushButton('Lower Height')
        trunk_lower_button.setStyleSheet("color: black; background-color: rgb(204, 153, 255)")
        trunkMoveGridLayout.addWidget(trunk_lower_button, 1, 0)
        trunk_lower_button.pressed.connect(self.armMove.clicked_lower_trunk)
        trunk_lower_button.setAutoRepeat(True)

        self.trunkMoveGrid.setLayout(trunkMoveGridLayout)
        self.trunkMoveGrid.setStyleSheet('QGroupBox:title {'
                                        'subcontrol-origin: margin;'
                                        'subcontrol-position: top center;'
                                        'padding-left: 10px;'
                                        'padding-right: 10px; }')

    def createArmMoveGrid(self):
        self.armMoveGrid = QGroupBox('Move Arm')
        armMoveGridLayout = QGridLayout()

        arm_raise_button = QPushButton('Raise Hand')
        arm_raise_button.setStyleSheet("color: black; background-color: rgb(204, 153, 255)")
        armMoveGridLayout.addWidget(arm_raise_button, 0, 0)
        arm_raise_button.clicked.connect(self.gripperMove.clicked_hand_gripper)
        arm_raise_button.clicked.connect(self.basePose.clicked_left_45)
        arm_raise_button.clicked.connect(self.headMove.clicked_right_45)
        arm_raise_button.clicked.connect(self.armMove.clicked_arm_raise)

        # arm_lower_button = QPushButton('Lower Hand')
        # armMoveGridLayout.addWidget(arm_lower_button, 1, 0)
        # arm_lower_button.clicked.connect(self.gripperMove.clicked_hand_gripper)
        # arm_lower_button.clicked.connect(self.basePose.clicked_right_45)
        # arm_lower_button.clicked.connect(self.headMove.clicked_left_45)
        # arm_lower_button.clicked.connect(self.armMove.clicked_arm_lower)

        body_neutral_button = QPushButton('Neutral Arm Position')
        body_neutral_button.setStyleSheet("color: black; background-color: rgb(204, 153, 255)")
        armMoveGridLayout.addWidget(body_neutral_button, 1, 0)
        body_neutral_button.clicked.connect(self.armMove.clicked_arm_neutral)
        body_neutral_button.clicked.connect(self.gripperMove.clicked_gripper_close)


        self.armMoveGrid.setLayout(armMoveGridLayout)
        self.armMoveGrid.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')
    def createMapGrid(self):
        self.mapBox = QGroupBox('Auto Drive')
        mapBoxLayout = QGridLayout()
        mapBoxLayout.addWidget(self.autodrive)
        self.mapBox.setLayout(mapBoxLayout)
        self.mapBox.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')

    def createCameraGrid(self):
        self.cameraBox = QGroupBox('')
        cameraBoxLayout = QGridLayout()

        self.rgblabel = QLabel()
        self.cameraFeed.changePixmap.connect(self.setStream)
        self.rgblabel.setAlignment(QtCore.Qt.AlignCenter)
        self.rgblabel.adjustSize()
        self.rgblabel.mousePressEvent = self.register_click_to_reach
        cameraBoxLayout.addWidget(self.rgblabel)
        self.cameraBox.setLayout(cameraBoxLayout)


    def createZoomPopGrid(self):
        self.zoomPopGrid = QGroupBox('')
        zoomPopGridLayout = QGridLayout()
        self.createCameraZoomGrid()
        self.createPopGrid()
        zoomPopGridLayout.addWidget(self.cameraZoomGrid, 0, 0)
        zoomPopGridLayout.addWidget(self.popGrid, 0, 1)
        self.zoomPopGrid.setLayout(zoomPopGridLayout)

    def openGrip(self):
        try:
            self.gripper.command(1.0)
        except:
            print('Failed to open gripper properly!')

    def closeGrip(self):
        try:
            self.gripper.grasp(-0.1)
        except:
            print('Failed to close gripper properly!')

    @QtCore.pyqtSlot(QPushButton)
    def buildHandControlPopup(self):
        popup_win = hand_control_ui.HandControlUI(self)
        popup_win.setGeometry(100, 200, 100, 100)
        popup_win.show()

    @QtCore.pyqtSlot(QPushButton)
    def buildHighResCamPopup(self):
        popup_win = high_res_cam_ui.StereoCamera(self)
        #popup_win.setGeometry(100, 200, 100, 100)
        popup_win.show()

    def createPopGrid(self):
        self.popGrid = QGroupBox('')
        popGridLayout = QGridLayout()

        # close_grip_button = QPushButton('Close Gripper')
        # gripGridLayout.addWidget(close_grip_button, 0, 0)
        # close_grip_button.pressed.connect(self.closeGrip)

        # open_grip_button = QPushButton('Open Gripper')
        # gripGridLayout.addWidget(open_grip_button, 1, 0)
        # open_grip_button.pressed.connect(self.openGrip)

        stereo_camera_button = QPushButton('High Resolution Camera')
        stereo_camera_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        popGridLayout.addWidget(stereo_camera_button, 0, 0)
        stereo_camera_button.pressed.connect(self.buildHighResCamPopup)

        save_button = QPushButton('Save Image')
        save_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        popGridLayout.addWidget(save_button, 1, 0)
        save_button.pressed.connect(self.cameraFeed.save_image)

        # grasp_control_button = QPushButton('Grasp Controls')
        # grasp_control_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        # popGridLayout.addWidget(grasp_control_button, 1, 0)


        # hand_controls_button = QPushButton('Open Hand Controls')
        # popGridLayout.addWidget(hand_controls_button, 1, 0)
        # hand_controls_button.pressed.connect(self.buildHandControlPopup)

        self.popGrid.setLayout(popGridLayout)
        self.popGrid.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')


    def createCameraZoomGrid(self):
        self.cameraZoomGrid = QGroupBox('Zoom')
        cameraZoomGridLayout = QGridLayout()

        plus_button = QPushButton('+')
        plus_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        cameraZoomGridLayout.addWidget(plus_button, 1, 0)
        plus_button.pressed.connect(self.cameraFeed.clicked_zoom_in)

        minus_button = QPushButton('-')
        minus_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        cameraZoomGridLayout.addWidget(minus_button, 2, 0)
        minus_button.pressed.connect(self.cameraFeed.clicked_zoom_out)

        self.cameraZoomGrid.setLayout(cameraZoomGridLayout)
        self.cameraZoomGrid.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')

    def pickUpSequence(self,pick_up,target_x,x_offset,target_y,y_offset,yaw_offset,height=None):
        try:
            if height < .5:
                desired_dist_from_obj = 0.449 # 17.5 inches
            else:
                desired_dist_from_obj = 0.500
            if self.autodrive.is_occupied(target_x+x_offset,target_y+y_offset):
                print "is occupied"
                dest = self.autodrive.calc_best_destination(target_x,target_y,desired_dist_from_obj,12)
                if dest is None:
                    self.speech.speak('Unable to reach destination')
                else:
                    self.speech.speak('Recalculating')
                    obj_bearing = math.atan2(target_y-dest[1],target_x-dest[0])
                    x_offset = -desired_dist_from_obj * math.cos(obj_bearing)
                    y_offset = -desired_dist_from_obj * math.sin(obj_bearing)
                    self.autodrive.go_to_mapXY(dest[0], 0, dest[1], 0, obj_bearing,yaw_offset)
            else:
                print "is not occupied"
                self.autodrive.go_to_mapXY(target_x, x_offset, target_y, y_offset,yaw_offset)    
        except:
            try:
                rospy.logerr('fail to reach near the target')
                print "trying other destinations"
                dest = self.autodrive.calc_best_destination(target_x,target_y,desired_dist_from_obj,12)
                if dest is None:
                    self.speech.speak('Unable to reach destination')
                else:
                    self.speech.speak('Recalculating')
                    obj_bearing = math.atan2(target_y-dest[1],target_x-dest[0])
                    x_offset = -desired_dist_from_obj * math.cos(obj_bearing)
                    y_offset = -desired_dist_from_obj * math.sin(obj_bearing)
                    self.autodrive.go_to_mapXY(dest[0], 0, dest[1], 0,yaw_offset, yaw=obj_bearing)
            except:
                print "Navigation failed a second time"
        if pick_up:
            print"picking up"
            self.gripperMove.clicked_gripper_open()
            self.armMove.auto_grasp_floor_object(height=height)
            print "closing gripper"
            self.gripperMove.clicked_gripper_close()
            print "done closing gripper"
            self.armMove.clicked_arm_neutral()

            current_pose = self.omni_base.get_pose()
            desired_dist_from_obj = 0.449 # 17.5 inches
            obj_bearing = math.atan2(-1.548-current_pose.pos.y,-1.704-current_pose.pos.x)
            x_offset = -desired_dist_from_obj * math.cos(obj_bearing)
            y_offset = -desired_dist_from_obj * math.sin(obj_bearing)
            thread.start_new_thread(self.pickUpSequence,(False,-1.704, x_offset, -1.548, y_offset,obj_bearing,-.17))
        else:
            print "putting down"
            self.armMove.auto_grasp_floor_object()
            self.gripperMove.clicked_gripper_open()
            self.armMove.clicked_arm_neutral()
            self.speech.speak('Here you go')
            print time.time()-self.timertime
            self.timertime = 0
            self.autodrive.recording = False

    def register_click_to_reach(self, event):
        #self.whole_body.end_effector_frame = u'hand_palm_link'
        rgb_h = event.pos().x()
        rgb_v = event.pos().y()
        print(rgb_h, rgb_v)
        if self.active_reach_target or self.active_put_down:
            finder3D = Get3Dcoordinates(rgb_h, rgb_v)
            while True:
                if finder3D.found_3d:
                    break
            target_x = finder3D.map_point.point.x
            target_y = finder3D.map_point.point.y
            target_z = finder3D.map_point.point.z
            print('rgbd frame: ', target_x, target_y, target_z)

            if math.isnan(target_x) or math.isnan(target_y) or math.isnan(target_z):
                print('The target location is not found (nan) by the RGBD sensor, try again!!')

            else:
                current_pose = self.omni_base.get_pose()
                if target_z < .5:
                    desired_dist_from_obj = 0.449 # 17.5 inches
                else:
                    desired_dist_from_obj = 0.500
                obj_bearing = math.atan2(target_y-current_pose.pos.y,target_x-current_pose.pos.x)
                x_offset = -desired_dist_from_obj * math.cos(obj_bearing)
                y_offset = -desired_dist_from_obj * math.sin(obj_bearing)

                # pickUpSequence
                if self.active_reach_target:
                    if target_z < .5:
                        thread.start_new_thread(self.pickUpSequence,(True,target_x, x_offset, target_y, y_offset,-.17,max(target_z-.05,0)))
                    else:
                        thread.start_new_thread(self.pickUpSequence,(True,target_x, x_offset, target_y, y_offset,-.17,min(target_z+.05,0.69)))
                else:
                    thread.start_new_thread(self.pickUpSequence,(False,target_x, x_offset, target_y, y_offset,-.17))
                del finder3D
                self.active_reach_target = False
                self.active_put_down = False

    def pointRandom(self):
        self.active_reach_target = True

    def putDown(self):
        self.active_put_down = True

    def retrieve(self):
        item_name = self.retrieveTextBox.text()
        if self.roam_classroom:
            location_probabilities,location_candidates = self.schemaNetwork.getLocationCandidates(str(item_name.strip()),True,'classroom')
        elif self.roam_breakroom:
            location_probabilities,location_candidates = self.schemaNetwork.getLocationCandidates(str(item_name.strip()),True,'breakroom')
        else:
            location_probabilities,location_candidates = self.schemaNetwork.getLocationCandidates(str(item_name.strip()),True,'both')
        location_probabilities = [1000,100,10,1,.1]
        sumprob = sum(location_probabilities)
        for i in range(len(location_probabilities)):
            location_probabilities[i] = float(location_probabilities[i])/sumprob
        #location_candidates = [[2.0,0.5],[1.0,-0.5],[0.0,-.5],[0.0,0.0],[0.0,1.0]] # random classroom
        #location_candidates = [[0.0,1.0],[0.0,1.0],[0.0,1.0],[0.0,1.0],[0.0,1.0]] # teddy bear
        #location_candidates = [[-3.5,-5.0],[-3.5,-5.0],[-3.5,-5.0],[-3.5,-5.0],[-3.5,-5.0]]
        thread.start_new_thread(self.roamSequence,(location_candidates,location_probabilities,True))
        self.timertime = time.time()
        #self.autodrive.recording = True

    def retrieveSequence(self):
        item_name = self.retrieveTextBox.text()
        item_shape,item_coordinates,item_time = self.cameraFeed.get_item_coordinates(item_name)
        if item_coordinates is None or (time.time()-item_time > 1):
            print 'Item not found'
            self.speech.speak('Searching for the ' + item_name)
            return False
        else:
            print '#######'
            print item_coordinates
            print item_shape
            #640,480
            print int(round(640*item_coordinates[0])), int(round(480*item_coordinates[1]))
            print '#######'
            finder3D = Get3Dcoordinates(int(round(640*item_coordinates[0])), int(round(480*item_coordinates[1])))
            while True:
                if finder3D.found_3d:
                    break
            target_x = finder3D.map_point.point.x
            target_y = finder3D.map_point.point.y
            target_z = finder3D.map_point.point.z
            print('rgbd frame: ', target_x, target_y, target_z)

            current_pose = self.omni_base.get_pose()

            if math.isnan(target_x) or math.isnan(target_y) or math.isnan(target_z):
                print('The target location is not found (nan) by the RGBD sensor, try again!!')
                self.speech.speak('The ' + item_name +' was not detected by my RGBD sensor')
                return False
            elif math.sqrt(math.pow(target_y-current_pose.pos.y,2)+math.pow(target_x-current_pose.pos.x,2)) > 2:
                print('The item is too far away for an accurate depth reading')
                self.speech.speak('The ' + item_name + ' is too far away for an accurate depth reading')
                return False
            elif target_z < .9: #.2
                if target_z < .5:
                    desired_dist_from_obj = 0.449 # 17.5 inches
                else:
                    desired_dist_from_obj = 0.500
                obj_bearing = math.atan2(target_y-current_pose.pos.y,target_x-current_pose.pos.x)
                x_offset = -desired_dist_from_obj * math.cos(obj_bearing)
                y_offset = -desired_dist_from_obj * math.sin(obj_bearing)

                # pickUpSequence
                self.speech.speak('Ok, I will get the '+item_name)
                if target_z < .5:
                    thread.start_new_thread(self.pickUpSequence,(True,target_x, x_offset, target_y, y_offset,-.17,max(target_z-.05,0)))
                else:
                    thread.start_new_thread(self.pickUpSequence,(True,target_x, x_offset, target_y, y_offset,-.17,min(target_z,0.69)))
                del finder3D
                #self.schemaNetwork.update_current_schema(item_name,[target_x,target_y])
                #self.schemaNetwork.train1PA(item_name,[target_x,target_y],True)

                return True
            else:
                print('Object is too high')
                self.speech.speak('The '+item_name+' is too high for me to reach!')
                return False
            return True

    def roam(self):
        if self.roam_classroom:
            thread.start_new_thread(self.roamSequence,(self.waypoints_classroom,self.probabilities_classroom,False))
        else:
            thread.start_new_thread(self.roamSequence,(self.waypoints_breakroom,self.probabilities_breakroom,False))

    def roamSequence(self,waypoints,probabilities,retrieving=False):
        self.autodrive.recording = True
        #if retrieving:
        #    if self.retrieveSequence():
        #        return
        self.schemaNetwork.current_schema = np.asarray([])
        waypoints_copy = copy.deepcopy(waypoints)
        probabilities_copy = copy.deepcopy(probabilities)
        print "PROBABILITIES"
        print probabilities
        sequence = self.roamTextBox.text()
        head_positions = [-3.839,-3.141,-2.443,-1.745,-1.047,-.349,.349,1.047,1.745]
        for i in range(5):
            # Pick random waypoint according to probability distribution
            random_index = np.random.choice(range(len(waypoints_copy)), 1, p=probabilities_copy)[0]
            random_waypoint = waypoints_copy[random_index]
            # Remove waypoint from list and renormalize probabilities
            del waypoints_copy[random_index]
            del probabilities_copy[random_index]
            sumprob = sum(probabilities_copy)
            for j in range(len(probabilities_copy)):
                probabilities_copy[j] = float(probabilities_copy[j])/sumprob
            # Go to waypoint and scan, pick up object if in retrieving mode
            try:
                desired_dist_from_obj = 0.800
                if self.autodrive.is_occupied(random_waypoint[0],random_waypoint[1]):
                    print "is occupied"
                    dest = self.autodrive.calc_best_destination(random_waypoint[0],random_waypoint[1],desired_dist_from_obj,12)
                    print("dest:")
                    print(dest)
                    if dest is None:
                        self.speech.speak('Unable to reach destination')
                    else:
                        self.speech.speak('Recalculating')
                        obj_bearing = math.atan2(random_waypoint[1]-dest[1],random_waypoint[0]-dest[0])
                        x_offset = -desired_dist_from_obj * math.cos(obj_bearing)
                        y_offset = -desired_dist_from_obj * math.sin(obj_bearing)
                        self.autodrive.go_to_mapXY(dest[0], 0.0, dest[1], 0.0, 0.0, obj_bearing)
                else:
                    print "is not occupied"
                    self.autodrive.go_to_mapXY(random_waypoint[0], 0.0, random_waypoint[1], 0.0,0.0)
            except:
                print "Navigation failed"
            for p in head_positions:
                if retrieving:
                    if self.retrieveSequence():
                        return
                else: # learning
                    self.cameraFeed.process_grabbable_objects()
                self.headMove.head_sweep(p)
        print time.time()-self.timertime
        if retrieving:
            self.speech.speak('Item not found')


    def createClick2ReachGrid(self):
        self.click2reachGrid = QGroupBox('')
        click2reachLayout = QGridLayout()

        click2pointbutton = QPushButton('Pick up object')
        click2pointbutton.setStyleSheet("color: white; background-color: rgb(0, 153, 153)")
        click2reachLayout.addWidget(click2pointbutton, 0, 0)
        click2pointbutton.pressed.connect(self.pointRandom)

        click2pointbutton = QPushButton('Put down object')
        click2pointbutton.setStyleSheet("color: white; background-color: rgb(0, 153, 153)")
        click2reachLayout.addWidget(click2pointbutton, 0, 1)
        click2pointbutton.pressed.connect(self.putDown)

        retrievebutton = QPushButton('Retrieve')
        retrievebutton.setStyleSheet("color: white; background-color: rgb(0, 153, 153)")
        click2reachLayout.addWidget(retrievebutton, 1, 1)
        retrievebutton.pressed.connect(self.retrieve,True)

        self.retrieveTextBox = QLineEdit()
        self.retrieveTextBox.setSizePolicy(QSizePolicy.Minimum,QSizePolicy.Minimum)
        click2reachLayout.addWidget(self.retrieveTextBox, 1, 0)

        roambutton = QPushButton('Roam Sequence')
        roambutton.setStyleSheet("color: white; background-color: rgb(0, 153, 153)")
        click2reachLayout.addWidget(roambutton, 2, 1)
        roambutton.pressed.connect(self.roam)

        self.roamTextBox = QLineEdit()
        self.roamTextBox.setSizePolicy(QSizePolicy.Minimum,QSizePolicy.Minimum)
        click2reachLayout.addWidget(self.roamTextBox, 2, 0)

        self.click2reachGrid.setLayout(click2reachLayout)
        self.click2reachGrid.setStyleSheet('QGroupBox:title {'
                                          'subcontrol-origin: margin;'
                                          'subcontrol-position: top center;'
                                          'padding-left: 10px;'
                                          'padding-right: 10px; }')


    def createHeadMoveGrid(self):
        self.headMoveGrid = QGroupBox('Move Head')
        headMoveGridLayout = QGridLayout()

        up_button = QPushButton('Up')
        up_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        headMoveGridLayout.addWidget(up_button, 0, 1)        
        up_button.pressed.connect(self.headMove.clicked_up)
        up_button.setAutoRepeat(True)

        left_button = QPushButton('<')
        left_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        headMoveGridLayout.addWidget(left_button, 1, 0)
        left_button.pressed.connect(self.headMove.clicked_left)
        left_button.setAutoRepeat(True)
        
        right_button = QPushButton('>')
        right_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        headMoveGridLayout.addWidget(right_button, 1, 2)
        right_button.pressed.connect(self.headMove.clicked_right)
        right_button.setAutoRepeat(True)
        
        down_button = QPushButton('Down')
        down_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        headMoveGridLayout.addWidget(down_button, 2, 1)
        down_button.pressed.connect(self.headMove.clicked_down)
        down_button.setAutoRepeat(True)

        home_button = QPushButton('Reset')
        home_button.setStyleSheet("color: white; background-color: rgb(0, 0, 255)")
        headMoveGridLayout.addWidget(home_button, 1, 1)
        home_button.clicked.connect(self.headMove.clicked_home)
        
        self.headMoveGrid.setLayout(headMoveGridLayout)
        self.headMoveGrid.setStyleSheet('QGroupBox:title {'
                 'subcontrol-origin: margin;'
                 'subcontrol-position: top center;'
                 'padding-left: 10px;'
                 'padding-right: 10px; }')

    def clicked_left_90(self):
        self.basePose.clicked_left_90()
        if not self.rotate_90_thread.isRunning():
            self.left_90_button.setEnabled(False)
            self.right_90_button.setEnabled(False)
            self.rotate_90_thread.start()

    def clicked_right_90(self):
        self.basePose.clicked_right_90()
        if not self.rotate_90_thread.isRunning():
            self.left_90_button.setEnabled(False)
            self.right_90_button.setEnabled(False)
            self.rotate_90_thread.start()

    def finish_90_thread(self):
        self.left_90_button.setEnabled(True)
        self.right_90_button.setEnabled(True)

    def createBaseMoveGrid(self):
        self.baseMoveGrid = QGroupBox('Move Base')
        baseMoveGridLayout = QGridLayout()

        forward_button = QPushButton('^')
        forward_button.setStyleSheet("color: black; background-color: rgb(153, 255, 153)")
        baseMoveGridLayout.addWidget(forward_button, 0, 1)
        forward_button.pressed.connect(self.baseMove.clicked_forward)
        forward_button.released.connect(self.baseMove.released)
        forward_button.setAutoRepeat(True)
        forward_button.setAutoRepeatDelay(0)

        left_button = QPushButton('<')
        left_button.setStyleSheet("color: black; background-color: rgb(153, 255, 153)")
        baseMoveGridLayout.addWidget(left_button, 1, 0)
        left_button.pressed.connect(self.baseMove.clicked_left)
        left_button.released.connect(self.baseMove.released)
        left_button.setAutoRepeat(True)

        right_button = QPushButton('>')
        right_button.setStyleSheet("color: black; background-color: rgb(153, 255, 153)")
        baseMoveGridLayout.addWidget(right_button, 1, 2)
        right_button.pressed.connect(self.baseMove.clicked_right)
        right_button.released.connect(self.baseMove.released)
        right_button.setAutoRepeat(True)

        backward_button = QPushButton('v')
        backward_button.setStyleSheet("color: black; background-color: rgb(153, 255, 153)")
        baseMoveGridLayout.addWidget(backward_button, 2, 1)
        backward_button.pressed.connect(self.baseMove.clicked_backward)
        backward_button.released.connect(self.baseMove.released)
        backward_button.setAutoRepeat(True)

        self.rotate_90_thread = Thread()
        self.rotate_90_thread.finished.connect(self.finish_90_thread)

        self.left_90_button = QPushButton('<<')
        self.left_90_button.setStyleSheet("color: black; background-color: rgb(30,205,50)")
        baseMoveGridLayout.addWidget(self.left_90_button, 2, 0)
        self.left_90_button.clicked.connect(self.clicked_left_90)

        self.right_90_button = QPushButton('>>')
        self.right_90_button.setStyleSheet("color: black; background-color: rgb(30,205,50)")
        baseMoveGridLayout.addWidget(self.right_90_button, 2, 2)
        self.right_90_button.clicked.connect(self.clicked_right_90)

        self.baseMoveGrid.setLayout(baseMoveGridLayout)
        self.baseMoveGrid.setStyleSheet('QGroupBox:title {'
                 'subcontrol-origin: margin;'
                 'subcontrol-position: top center;'
                 'padding-left: 10px;'
                 'padding-right: 10px; }')

    def shutdown(self):
        rospy.loginfo("Stopping the robot.....")
        self.baseMove.shutdown()
        self.headMove.shutdown()
        self.basePose.shutdown()

def main():
    app = QApplication(sys.argv)
    ex = ClientUI()
    rospy.on_shutdown(ex.shutdown)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
