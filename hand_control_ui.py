from PyQt5.QtWidgets import QDialog, QGridLayout, QGroupBox, QPushButton, QLabel

class HandControlUI(QDialog):

    def __init__(self, parent = None):
        super(HandControlUI, self).__init__(parent)
        self.title = 'HSR hand Control'
        self.left = 100
        self.top = 100
        self.width = 1150
        self.height = 700
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.setFixedSize(self.width, self.height)
        self.createGridLayout()
        self.show()

    def createGridLayout(self):
        pass