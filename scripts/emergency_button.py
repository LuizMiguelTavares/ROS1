#!/usr/bin/env python3

import os
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSignal, QTimer
import rospy
from std_msgs.msg import Bool

class EmergencyButtonNode(object):
    def __init__(self):
        rospy.init_node('emergency_button_node', anonymous=True)

        self.data = False

        self.count = 0

        self.publisher = rospy.Publisher('emergency_flag', Bool, queue_size=10)
        rospy.loginfo('Emergency button node started')

    def publishEmergencySignal(self):
        # Method to publish the emergency signal

        # Set the data value to True
        self.data = True
        
        # Log the emergency stop message
        rospy.loginfo('Emergency STOP sent!')
    
    def pub(self):
        # Method to publish the data periodically

        if self.data == True:
            # Check if emergency stop is active
            self.count += 1
            if self.count > 10:
                # Exit the program if emergency stop is active for more than 10 cycles
                rospy.signal_shutdown("Emergency Stop Activated")
    
        # Create a Bool message
        msg = Bool()
        msg.data = self.data

        # Publish the message
        self.publisher.publish(msg)

class EmergencyButton(QWidget):
    # Custom QWidget class for the emergency button

    emergency_signal = pyqtSignal()

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.initUI()

        # Create a QTimer for periodic updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.node.pub)
        self.timer.start(int(1000/30))  # 1/30 sec in milliseconds

    def initUI(self):
        # Initialize the UI

        self.setWindowTitle('Emergency Button')
        self.setGeometry(600, 600, 500, 500)

        # Set the path to the icon file
        icon_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'emergency_icon.jpg')

        # Create the emergency button
        btn_emergency = QPushButton(self)
        btn_emergency.setGeometry(50, 50, 400, 400)
        btn_emergency.setIcon(QIcon(icon_path))
        btn_emergency.setIconSize(btn_emergency.size())
        btn_emergency.clicked.connect(self.emergency_signal.emit)

        # Connect the button's clicked signal to emit the emergency signal
        self.emergency_signal.connect(self.node.publishEmergencySignal)

def main():
    # Create the emergency button node
    button_node = EmergencyButtonNode()

    app = QApplication([])

    # Create the emergency button widget
    button_widget = EmergencyButton(button_node)
    button_widget.show()

    # Start the PyQt event loop
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
