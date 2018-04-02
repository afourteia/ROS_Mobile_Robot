
import os
import time

import rospy
import rospkg


from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class MotorController(QWidget):
    def __init__(self):
        '''
        This is the main screen that the user interacts with.

        '''
        super(MotorController, self).__init__()
        # Give QObjects reasonable names
        print("I am here")
        rp = rospkg.RosPack()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        self.release_motor_button.clicked.connect(self.releaseMotorcmd)
        # self.beep_button.clicked.connect(self.beepcmd)
        # self.activate_joy_button.clicked.connect(self.activateJoycmd)
        # self.activate_follow_mode_button.clicked.connect(self.followModecmd)
        # self.calibrate_IMU_button.clicked.connect(self.calibrateIMUcmd)
        # self.enter_wire_control_mode_button.clicked.connect(self.enterWireCTRLcmd)
        # self.exit_wire_control_mode_button.clicked.connect(self.exitWireCTRLcmd)
        # self.clear_error_messages_button.clicked.connect(self.clearErrorcmd)
        #

        # timer = QtCore.QTimer(self)
        # timer.timeout.connect(self.updateFromGlobal)
        # timer.start(100)

    def releaseMotorcmd(self):
        print "update"

    # def beepcmd(self):
    #     print "Joystick button pressed"
    #
    # def activateJoycmd(self):
    #     print "Follow button pressed"
    #
    # def followModecmd(self):
    #     print "Beep button pressed"
    #
    # def calibrateIMUcmd(self):
    #     print "Motor Release button pressed"
    #
    # def enterWireCTRLcmd(self,value):
    #     global something
    #     #publish from in here
    #     print "left setpoint slider moved"
    #
    # def exitWireCTRLcmd(self,value):
    #     print "right setpoint slider moved"
    #
    # def clearErrorcmd(self,value):
    #     print "left actual slider moved"

# app = QtWidgets.QApplication(sys.argv)  # A new instance of QApplication
# form = MainScreen()  # We set the form to be our ExampleApp (design)
# form.show()  # Show the form
# app.exec_()  # and execute the app
