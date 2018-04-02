import os
import rospy
import rospkg

import std_msgs.msg
from controller_box.msg import UKARTdiag
from controller_box.msg import UKARTparams

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtWidgets import QShortcut, QWidget

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')
        rp = rospkg.RosPack()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rp.get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Add publisher
        self._publisher = rospy.Publisher("testing", std_msgs.msg.String)
        self._subscriber = rospy.Subscriber("chatter", std_msgs.msg.String, self.subscriber_cb)

        # setup callback functions for buttons
        self._widget.release_motor_button.clicked.connect(self.releaseMotorcmd)

        # Add some style sheet
        self.flagged    = """ QLabel { background-color: rgb(255, 0, 0);}"""
        self.unflagged  = """ QLabel { background-color: rgb(0, 255, 0);}"""

        # setup timer
        # self._timer = QTimer()
        # self._timer.timeout.connect(self._update_message_state)
        # self._timer.start(100)

    def releaseMotorcmd(self):
        print "update"
        self._publisher.publish("HELLOOO")

    def subscriber_cb(self, data):
        print "Done"
        if (data.data == "HI"):
            self._widget.bus_error.setStyleSheet(self.flagged)
        else:
            self._widget.bus_error.setStyleSheet(self.unflagged)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
