#!/usr/bin/env python

""" Testing the mock class """

PKG = 'pandora_behave'

import rospy

import roslib

from mock.mock import Mock

roslib.load_manifest(PKG)


if __name__ == '__main__':

    QRMock = Mock('qr_mock')

    rospy.spin()

