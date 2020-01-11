from PyQt5.QtWidgets import QDialog, QGridLayout, QGroupBox, QPushButton, QLabel
from PyQt5 import QtGui, QtCore
import stereo_cam_channels

class StereoCamera(QDialog):

    def __init__(self, parent = None):
        super(StereoCamera, self).__init__(parent)
        self.title = 'HSR High Resolution Camera'
        self.left = 50
        self.top = 0
        self.width = 1300
        self.height = 1000
        self.cameraFeed = stereo_cam_channels.rgbOut()
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.setFixedSize(self.width, self.height)
        self.createGridLayout()
        self.show()

    def createGridLayout(self):
        self.createCameraGrid()
        self.createCameraZoomGrid()
        gridLayout = QGridLayout()
        gridLayout.addWidget(self.cameraBox, 0, 0)
        gridLayout.addWidget(self.cameraZoomGrid, 1, 0)
        self.setLayout(gridLayout)

    @QtCore.pyqtSlot(QtGui.QImage)
    def setStream(self, image):
        self.rgblabel.setPixmap(QtGui.QPixmap.fromImage(image))

    def createCameraGrid(self):
        self.cameraBox = QGroupBox('')
        cameraBoxLayout = QGridLayout()

        self.rgblabel = QLabel()
        self.cameraFeed.changePixmap.connect(self.setStream)
        self.rgblabel.setAlignment(QtCore.Qt.AlignCenter)
        self.rgblabel.adjustSize()
        cameraBoxLayout.addWidget(self.rgblabel)
        self.cameraBox.setLayout(cameraBoxLayout)

    def createCameraZoomGrid(self):
        self.cameraZoomGrid = QGroupBox('Zoom')
        cameraZoomGridLayout = QGridLayout()

        plus_button = QPushButton('+')
        plus_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        cameraZoomGridLayout.addWidget(plus_button, 0, 0)
        plus_button.pressed.connect(self.cameraFeed.clicked_zoom_in)

        minus_button = QPushButton('-')
        minus_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        cameraZoomGridLayout.addWidget(minus_button, 1, 0)
        minus_button.pressed.connect(self.cameraFeed.clicked_zoom_out)

        self.cameraZoomGrid.setLayout(cameraZoomGridLayout)
        self.cameraZoomGrid.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')