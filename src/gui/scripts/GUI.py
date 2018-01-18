# -*- coding: utf-8 -*-
"""
Created on Wed Jan 17 17:24:48 2018

@author: joell
"""
from PyQt5 import QtCore, QtGui, uic  # Import the PyQt4 module we'll need
import sys  # We need sys so that we can pass argv to QApplication


something = 1

#suscribe from out here and write to a local variable to update on the screen

class MainScreen(QtGui.QMainWindow):
    def __init__(self):
        global something
        '''
        This is the main screen that the user interacts with.

        '''
        super(self.__class__, self).__init__()
        uic.loadUi('TestingScreen.ui', self)

        self.pushButton.clicked.connect(self.joystickButton)
        self.pushButton_2.clicked.connect(self.followModeButton)
        self.pushButton_3.clicked.connect(self.beepButton)
        self.pushButton_4.clicked.connect(self.motorReleaseButton)

        self.verticalSlider.sliderMoved.connect(self.leftSetpointSlider)
        self.verticalSlider_2.sliderMoved.connect(self.rightSetpointSlider)
        self.verticalSlider_3.sliderMoved.connect(self.leftActualtSlider)
        self.verticalSlider_4.sliderMoved.connect(self.rightActualSlider)


        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.updateFromGlobal)
        timer.start(100)

    def updateFromGlobal(self):
        print "update"
    def joystickButton(self):
        print "Joystick button pressed"
    def followModeButton(self):
        print "Follow button pressed"
    def beepButton(self):
        print "Beep button pressed"
    def motorReleaseButton(self):
        print "Motor Release button pressed"
    def leftSetpointSlider(self,value):
        global something
        #publish from in here
        print "left setpoint slider moved"
    def rightSetpointSlider(self,value):
        print "right setpoint slider moved"
    def leftActualtSlider(self,value):
        print "left actual slider moved"
    def rightActualSlider(self,value):
        print "right actual slider moved"


# start main ros nodes here
app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
form = MainScreen()  # We set the form to be our ExampleApp (design)
form.show()  # Show the form
app.exec_()  # and execute the app
