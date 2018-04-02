
from PyQt5 import QtCore, QtGui, uic, QtWidgets
import sys


class MainScreen(QtWidgets.QMainWindow):
    def __init__(self):
        '''
        This is the main screen that the user interacts with.

        '''
        super(self.__class__, self).__init__()
        uic.loadUi('TestingScreen.ui', self)

        self.release_motor_button.clicked.connect(self.releaseMotorcmd)
        self.beep_button.clicked.connect(self.beepcmd)
        self.activate_joy_button.clicked.connect(self.activateJoycmd)
        self.activate_follow_mode_button.clicked.connect(self.followModecmd)
        self.calibrate_IMU_button.clicked.connect(self.calibrateIMUcmd)
        self.enter_wire_control_mode_button.clicked.connect(self.enterWireCTRLcmd)
        self.exit_wire_control_mode_button.clicked.connect(self.exitWireCTRLcmd)
        self.clear_error_messages_button.clicked.connect(self.clearErrorcmd)


        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.updateFromGlobal)
        timer.start(100)

    def releaseMotorcmd(self):
        print "update"

    def beepcmd(self):
        print "Joystick button pressed"

    def activateJoycmd(self):
        print "Follow button pressed"

    def followModecmd(self):
        print "Beep button pressed"

    def calibrateIMUcmd(self):
        print "Motor Release button pressed"

    def enterWireCTRLcmd(self,value):
        global something
        #publish from in here
        print "left setpoint slider moved"

    def exitWireCTRLcmd(self,value):
        print "right setpoint slider moved"

    def clearErrorcmd(self,value):
        print "left actual slider moved"

def startGUI():
    app = QtWidgets.QApplication(sys.argv)  # A new instance of QApplication
    form = MainScreen()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    app.exec_()  # and execute the app

def initROS():


if(__name__=="__main__"):

    initROS()
    startGUI()
