import sys
from PyQt5.QtWidgets import QDialog, QGridLayout, QGroupBox, QPushButton, QLabel, QApplication, QVBoxLayout, QHBoxLayout, QLineEdit
from PyQt5 import QtGui, QtCore

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import numpy as np

import random

class NetworkUI(QDialog):
    def __init__(self, parent=None):
        super(NetworkUI, self).__init__(parent)

        self.title = 'Schema Network Interface'
        self.left = 500
        self.top = 100
        self.width = 1400
        self.height = 900
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        #self.setFixedSize(self.width, self.height)
        
        # Network display
        self.figure = Figure(figsize=(8,10))
        self.canvas = FigureCanvas(self.figure)

        # Network info
        self.info = QLabel('Current Trial: 0 \nSchema Objects: \nSchema: \nLC_Max: ')

        # Controls
        self.controlsBox = QGroupBox()
        self.controlsLayout = QHBoxLayout()
        self.buttonPlot = QPushButton('Consolidate')
        #self.buttonPlot.clicked.connect(self.plot)
        self.buttonSave = QPushButton('Save')
        self.buttonLoad = QPushButton('Load')
        self.textFilename = QLineEdit()
        self.controlsLayout.addWidget(self.buttonPlot)
        self.controlsLayout.addWidget(self.buttonSave)
        self.controlsLayout.addWidget(self.buttonLoad)
        self.controlsLayout.addWidget(self.textFilename)
        self.controlsBox.setLayout(self.controlsLayout)

        # set the layout
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(self.info)
        layout.addWidget(self.controlsBox)
        self.setLayout(layout)

    def plot(self):
        ''' plot some random stuff '''
        # random data
        data = [random.random() for i in range(10)]

        # create an axis
        ax = self.figure.add_subplot(111)

        # discards the old graph
        ax.clear()

        # plot data
        ax.plot(data, '*-')

        # refresh canvas
        self.canvas.draw()

    def update_info(self,schema_objects,schema,lc_max):
        schema_objects_string = ''
        for so in schema_objects:
            schema_objects_string = schema_objects_string + so + ' '
        self.info.setText('Current Trial: 0 \nSchema Objects: '+schema_objects_string+'\nSchema:' + str(schema) + '\nLC_Max: ' + str(lc_max))

    def visualize(self,n,w):
        self.figure.clf()
        n_context = n['n_context']
        n_flavor = n['n_flavor']
        n_multimodal = n['n_multimodal']
        n_well = n['n_well']
        n_context_pattern = n['n_context_pattern']
        n_vhipp = n['n_vhipp']
        n_dhipp = n['n_dhipp']
        familiarity = n['familiarity']
        noveltyy = n['noveltyy']
        w_pattern_context = w['w_pattern_context']
        w_context_multimodal = w['w_context_multimodal']
        w_flavor_multimodal = w['w_flavor_multimodal']
        w_multimodal_well = w['w_multimodal_well']
        w_vhipp_multimodal = w['w_vhipp_multimodal']
        w_well_dhipp = w['w_well_dhipp']
        w_flavor_dhipp = w['w_flavor_dhipp']
        w_vhipp_dhipp = w['w_vhipp_dhipp']
        w_context_vhipp = w['w_context_vhipp']
        w_context_familiarity = w['w_context_familiarity']
        w_dhipp_novelty = w['w_dhipp_novelty']
        
        gs = gridspec.GridSpec(5, 10,width_ratios=[1,4,1,4,1,4,1,1,1,1],height_ratios=[4,1,4,4,4])
    
        ax_context = self.figure.add_subplot(gs[22])
        ax_context.set_title('mPFC')
        ax_context.get_xaxis().set_ticks([])
        ax_context.get_yaxis().set_ticks([])
        ax_flavor = self.figure.add_subplot(gs[42])
        ax_flavor.set_title('cue')
        ax_flavor.get_xaxis().set_ticks([])
        ax_flavor.get_yaxis().set_ticks([])
        ax_multimodal = self.figure.add_subplot(gs[44])
        ax_multimodal.set_title('AC')
        ax_multimodal.get_xaxis().set_ticks([])
        ax_multimodal.get_yaxis().set_ticks([])
        ax_well = self.figure.add_subplot(gs[46])
        ax_well.set_title('action')
        ax_well.get_xaxis().set_ticks([])
        ax_well.get_yaxis().set_ticks([])
        ax_cp = self.figure.add_subplot(gs[20])
        ax_cp.set_title('pattern')
        ax_cp.get_xaxis().set_ticks([])
        ax_cp.get_yaxis().set_ticks([])
        ax_vhipp = self.figure.add_subplot(gs[24])
        ax_vhipp.set_title('vHPC')
        ax_vhipp.get_xaxis().set_ticks([])
        ax_vhipp.get_yaxis().set_ticks([])
        ax_dhipp = self.figure.add_subplot(gs[26])
        ax_dhipp.set_title('dHPC')
        ax_dhipp.get_xaxis().set_ticks([])
        ax_dhipp.get_yaxis().set_ticks([])
        ax_familiarity = self.figure.add_subplot(gs[5])
        ax_familiarity.set_title('familiarity')
        ax_familiarity.get_xaxis().set_ticks([])
        ax_familiarity.get_yaxis().set_ticks([])
        ax_noveltyy = self.figure.add_subplot(gs[3])
        ax_noveltyy.set_title('novelty')
        ax_noveltyy.get_xaxis().set_ticks([])
        ax_noveltyy.get_yaxis().set_ticks([])
        ax_p_c = self.figure.add_subplot(gs[21])
        ax_p_c.set_title('pattern to mPFC')
        ax_p_c.get_xaxis().set_ticks([])
        ax_p_c.get_yaxis().set_ticks([])
        ax_c_m = self.figure.add_subplot(gs[33])
        ax_c_m.set_title('mPFC to AC')
        ax_c_m.get_xaxis().set_ticks([])
        ax_c_m.get_yaxis().set_ticks([])
        ax_f_m = self.figure.add_subplot(gs[43])
        ax_f_m.set_title('cue to AC')
        ax_f_m.get_xaxis().set_ticks([])
        ax_f_m.get_yaxis().set_ticks([])
        ax_m_w = self.figure.add_subplot(gs[45])
        ax_m_w.set_title('AC to action')
        ax_m_w.get_xaxis().set_ticks([])
        ax_m_w.get_yaxis().set_ticks([])
        ax_v_m = self.figure.add_subplot(gs[34])
        ax_v_m.set_title('vHPC to AC')
        ax_v_m.get_xaxis().set_ticks([])
        ax_v_m.get_yaxis().set_ticks([])
        ax_x_d = self.figure.add_subplot(gs[25])
        ax_x_d.set_title('vHPC, cue, action to dHPC')
        ax_x_d.get_xaxis().set_ticks([])
        ax_x_d.get_yaxis().set_ticks([])
        ax_c_v = self.figure.add_subplot(gs[23])
        ax_c_v.set_title('mPFC to vHPC')
        ax_c_v.get_xaxis().set_ticks([])
        ax_c_v.get_yaxis().set_ticks([])
        ax_c_f = self.figure.add_subplot(gs[15])
        ax_c_f.set_title('mPFC to familiarity')
        ax_c_f.get_xaxis().set_ticks([])
        ax_c_f.get_yaxis().set_ticks([])
        ax_d_n = self.figure.add_subplot(gs[13])
        ax_d_n.set_title('dHPC to novelty')
        ax_d_n.get_xaxis().set_ticks([])
        ax_d_n.get_yaxis().set_ticks([])
    
        ax_context.imshow(n_context,interpolation='none',aspect='auto')
        ax_flavor.imshow(n_flavor,interpolation='none',aspect='auto')
        ax_multimodal.imshow(n_multimodal,interpolation='none',aspect='auto')
        ax_well.imshow(n_well,interpolation='none',aspect='auto')
        ax_cp.imshow(n_context_pattern,interpolation='none',aspect='auto')
        ax_vhipp.imshow(n_vhipp,interpolation='none',aspect='auto')
        ax_dhipp.imshow(n_dhipp,interpolation='none',aspect='auto')
        ax_familiarity.imshow(familiarity,interpolation='none',aspect='auto')
        ax_noveltyy.imshow(noveltyy,interpolation='none',aspect='auto')
        ax_p_c.imshow(w_pattern_context,interpolation='none',aspect='auto')
        ax_c_m.imshow(w_context_multimodal,interpolation='none',aspect='auto')
        ax_f_m.imshow(w_flavor_multimodal,interpolation='none',aspect='auto')
        ax_m_w.imshow(w_multimodal_well,interpolation='none',aspect='auto')
        ax_v_m.imshow(w_vhipp_multimodal,interpolation='none',aspect='auto')
        ax_x_d.imshow(np.concatenate((w_well_dhipp,w_flavor_dhipp,w_vhipp_dhipp),axis=1),interpolation='none',aspect='auto')
        ax_c_v.imshow(w_context_vhipp,interpolation='none',aspect='auto')
        ax_c_f.imshow(w_context_familiarity,interpolation='none',aspect='auto')
        ax_d_n.imshow(w_dhipp_novelty,interpolation='none',aspect='auto')
        # refresh canvas
        self.canvas.draw()

if __name__ == '__main__':
    app = QApplication(sys.argv)

    main = NetworkUI()
    main.show()

    sys.exit(app.exec_())