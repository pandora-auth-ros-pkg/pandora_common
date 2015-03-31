#!/usr/bin/env python

""" ROS Test for data fusion """

PKG = 'pandora_behave'

import os
import rospy
import roslib
roslib.load_manifest(PKG)

from mock.coordinator import Coordinator
from pandora_behave.msg import MockCmd


TIMEOUT = 30

if __name__ == '__main__':

    msg = MockCmd()
    msg.input_topic = '/camera/image_raw'
    msg.cmd = 'alert'
    msg.alert_delay = 0.6
    msg.debug = True

    master = Coordinator('master')
    master.register_reporter('accuracy tester', '/vision/qr_alert', 'QrAlertsVectorMsg')
    master.register_mock('hole_mock')

    master.set_state(2)

    master.send_command('hole_mock', msg)

    rospy.sleep(TIMEOUT)

    master.shutdown('Master is shutting down', with_mocks=True)




