#!/usr/bin/env python

""" A Vision Hole mock """

PKG = 'pandora_behave'

import rospy

import roslib

from mock.mock import Mock

roslib.load_manifest(PKG)


if __name__ == '__main__':

    HoleMock = Mock('hole_mock')

    rospy.spin()

