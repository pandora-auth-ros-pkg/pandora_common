#!/usr/bin/env python

""" Using coordinator """

PKG = 'pandora_behave'

import rospy
import roslib
roslib.load_manifest(PKG)

from mock.coordinator import Coordinator
from pandora_behave.msg import MockCmd


TIMEOUT = 10

if __name__ == '__main__':

    msg = MockCmd()
    msg.input_topic = '/vision/qr_alert'
    msg.cmd = 'alert'
    msg.alert_type = 'QRAlertsVectorMsg'
    msg.yaw = 2
    msg.pitch = 1
    msg.alert_delay = 1
    msg.alert_repetitions = 6
    msg.probability = 1
    msg.QRcontent = 'random QR'
    msg.debug = True

    master = Coordinator('master', with_state_client=False)

    # master.register_mock('hole_mock')

    master.register_reporter('counter',
                             '/data_fusion/world_model', 'WorldModelMsg')

    MockResponse = master.send_command('hole_mock', msg)
    response = MockResponse.response

    if response.done:
        rospy.loginfo('Time spent: ' + str(response.time_taken))

    rospy.loginfo('Data fusion sent: ')
    rospy.loginfo(master.reporter('counter').data)

    shutdown_response = master.shutdown('Master is shutting down',
                                        with_mocks=False)

    if shutdown_response.done:
        rospy.loginfo(shutdown_response.time_taken)
