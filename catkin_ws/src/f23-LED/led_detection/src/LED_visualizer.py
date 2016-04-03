#!/usr/bin/env python
import rospy, sys
from led_detection.LEDDetector import LEDDetector
from duckietown_msgs.msg import Vector2D, LEDDetection, LEDDetectionArray, LEDDetectionDebugInfo
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from math import sqrt

from sensor_msgs.msg import CompressedImage 
from duckietown_utils.bag_logs import numpy_from_ros_compressed
import numpy as np

# plotting 
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar


## Aux
gray_color_table = [qRgb(i, i, i) for i in range(256)]

def toQImage(im, copy=False):
    if im is None:
        return QImage()

    if im.dtype == np.uint8:
        if len(im.shape) == 2:
            qim = QImage(im.data, im.shape[1], im.shape[0], im.strides[0], QImage.Format_Indexed8)
            qim.setColorTable(gray_color_table)
            return qim.copy() if copy else qim

        elif len(im.shape) == 3:
            if im.shape[2] == 3:
                qim = QImage(im.data, im.shape[1], im.shape[0], im.strides[0], QImage.Format_RGB888);
                return qim.copy() if copy else qim
            elif im.shape[2] == 4:
                qim = QImage(im.data, im.shape[1], im.shape[0], im.strides[0], QImage.Format_ARGB32);
                return qim.copy() if copy else qim

    raise NotImplementedException

# For multithread-safe
QCoreApplication.setAttribute(Qt.AA_X11InitThreads)

class plotWin(QDialog):
    def __init__(self, parent = None):
        super(plotWin, self).__init__(parent)
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)# set the layout
        self.toolbar = NavigationToolbar(self.canvas, self)

        layout = QVBoxLayout()
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

class LEDWindow(QWidget):
    progress = pyqtSignal(bool, float)
    
    def __init__(self, parent = None):
        super(LEDWindow, self).__init__(parent)
        self.createLayout()
        self.camera_image = None
        self.variance_map = None
        self.unfiltered_leds = None
        self.filtered_leds = None
        self.cell_size = None
        self.camtl = [0, 40] # top left
        self.progress.connect(self.updateBar)
        self.figDialogs = []
        self.plotCamImage = True
        self.imagescale = 0.0

    def updateDebugInfo(self, msg):
        if(len(msg.variance_map.data)):
            self.variance_map = toQImage(numpy_from_ros_compressed(msg.variance_map))
        self.progress.emit(msg.capturing, msg.capture_progress)
        self.unfiltered_leds = msg.led_all_unfiltered
        self.cell_size = msg.cell_size

    def updateBar(self, active, progr):
        self.progressBar.setEnabled(active)
        self.progressBar.setValue(progr)

    def updateResults(self, msg):
        self.filtered_leds = msg

    def createLayout(self):
        self.setWindowTitle("LED Detector Debug Visualizer")

        progressLabel = QLabel("Capture:")
        self.progressBar = QProgressBar()
        self.canvas = QWidget()
        addressEdit = QTextEdit()

        # Put the widgets in a layout (now they start to appear):
        layout = QGridLayout()
        layout.addWidget(progressLabel, 0, 0)
        layout.addWidget(self.progressBar, 0, 1)
        layout.addWidget(self.canvas, 1, 0)

        layout.setRowStretch(2, 1)
        self.setLayout(layout)

        self.resize(640, 540)

    def paintEvent(self, event):
        qp = QPainter()
        qp.begin(self)
        pen = QPen()
        pen.setWidth(3);
        pen.setBrush(QColor(255, 0, 0));
        qp.setPen(pen)
        font = QFont()
        font.setPixelSize(16)
        qp.setFont(font)

        win_size = [self.frameGeometry().width()-self.camtl[0]-20,
                    self.frameGeometry().height()-self.camtl[1]-50]

        image = self.camera_image if self.plotCamImage else self.variance_map

        
        if(self.camera_image is not None):
            self.imagescale = min(1.0*win_size[0]/self.camera_image.width(),
                        1.0*win_size[1]/self.camera_image.height())
            qp.translate(self.camtl[0]+0.5*(win_size[0]-self.imagescale*self.camera_image.width())
                         , self.camtl[1]+0.5*(win_size[1]-self.imagescale*self.camera_image.height()))

        if(image is not None):
            #print('scale:%s'%imagescale)
            qp.scale(self.imagescale,self.imagescale)
            k = 1.0*self.camera_image.width()/image.width()
            qp.scale(k,k)
            qp.drawImage(0,0,image)
            qp.scale(1.0/k,1.0/k)

        if(self.unfiltered_leds is not None):
            for led in self.unfiltered_leds.detections:
                led_rect = QRect(led.pixels_normalized.x-0.5*self.cell_size[0],
                                 led.pixels_normalized.y-0.5*self.cell_size[1], 
                                 self.cell_size[0],
                                 self.cell_size[1])
                qp.drawText(led.pixels_normalized.x-0.5*self.cell_size[0],
                            led.pixels_normalized.y-0.5*self.cell_size[1]-10,
                            QString.number(led.frequency, 'g', 2))
                qp.drawRect(led_rect)

        pen.setBrush(QColor(0, 255, 0));
        qp.setPen(pen)

        if(self.filtered_leds is not None):
            for led in self.filtered_leds.detections:
                led_rect = QRect(led.pixels_normalized.x-0.5*self.cell_size[0],
                                 led.pixels_normalized.y-0.5*self.cell_size[1], 
                                 self.cell_size[0],
                                 self.cell_size[1])
                qp.drawText(led.pixels_normalized.x+0.5*self.cell_size[0]+10,
                            led.pixels_normalized.y-0.5*self.cell_size[1],
                            QString.number(led.frequency, 'g', 2))
                            #1.0*led.frequency)
                qp.drawRect(led_rect)
        qp.end()

    def mousePressEvent(self, event):
        click_img_coord = event.pos()-QPoint(self.camtl[0], self.camtl[1])
        click_img_coord = 1.0*click_img_coord/self.imagescale
        mindist = float("inf")
        closest = None
        if(self.unfiltered_leds):
            for d in self.unfiltered_leds.detections: 
                dist = (d.pixels_normalized.x-click_img_coord.x())**2 + (d.pixels_normalized.y-click_img_coord.y())**2
                if(dist < mindist and 
                   dist < self.imagescale**2*self.cell_size[0]**2+self.cell_size[1]**2):
                    closest = d
                    mindist = dist
        if closest is not None:
            self.figDialogs.append(plotWin())
            self.plot(closest, self.figDialogs[-1].figure)
            self.figDialogs[-1].canvas.draw()
            self.figDialogs[-1].show()
        else:
            # Switch from camera image to variance map
            self.plotCamImage = not self.plotCamImage 
            if(not self.plotCamImage and self.variance_map is None):
                self.plotCamImage = True


    def plot(self, closest, figure):
        ax = figure.add_subplot(211)
        #print("Timestamps: {0}".format(closest.signal_ts))
        ax.plot(closest.signal_ts, closest.signal)
        ax.set_title('Signal @ ('+str(closest.pixels_normalized.x)+\
                      ', ' + str(closest.pixels_normalized.y) + ')', fontsize=12)#, fontweight='bold')
        ax2 = figure.add_subplot(212)
        ax2.plot(closest.fft_fs,closest.fft)
        ax2.set_title('FFT @ ('+str(closest.pixels_normalized.x)+\
                      ', ' + str(closest.pixels_normalized.y)+ ')', fontsize=12)#, fontweight='bold')

app = QApplication(sys.argv)
win = LEDWindow(None)

class LEDVisualizerNode(object):
    def __init__(self):
        self.first_timestamp = None
        self.data = []
        self.capture_time = 2.3 # capture time
        self.capture_finished = False
        self.tinit = None

        self.sub_info = rospy.Subscriber("/LED_detector_node/debug_info", LEDDetectionDebugInfo, self.info_callback)
        self.sub_info = rospy.Subscriber("/maserati/camera_node/image/compressed", CompressedImage, self.cam_callback)
        self.sub_info = rospy.Subscriber("/LED_detector_node/raw_led_detection", LEDDetectionArray, self.result_callback)

    def info_callback(self, msg):
        #print('Received info')
        win.updateDebugInfo(msg)

    def result_callback(self, msg):
        win.updateResults(msg)

    def cam_callback(self, msg):
        #print('Received camera image')
        npimg = numpy_from_ros_compressed(msg)
        win.camera_image = toQImage(npimg)
        win.update()
        
rospy.init_node('LED_visualizer_node',anonymous=False)
node = LEDVisualizerNode()
#rospy.spin() # not quite needed for callbacks in python?
win.show()
sys.exit(app.exec_())
