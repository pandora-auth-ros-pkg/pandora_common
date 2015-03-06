# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2014, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Houtas Vasileios, Tsirigotis Christos"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"

import os
import unittest

import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from pandora_testing_tools.testing_interface import test_base
from sensor_msgs.msg import Image

class VisionTestBase(test_base.TestBase):
    def readImages(self, imagePath):
        self.images = []
        imageTypes = [".png", ".jpg", ".bmp"]
        bridge = CvBridge()
        if not os.path.isdir(imagePath):
            print "ERROR : Incorrect Path"
            exit(1)

        for fileName in os.listdir(imagePath):
            isImage = False
            for type in imageTypes:
                if (type in fileName):
                    isImage = True
                    break
            if (not isImage):
                print fileName, "is not an image"
                continue
            # Read the next image.
            currentImg = cv2.imread(os.path.join(imagePath,
                                    fileName), -1)

            # If the image was not read succesfully continue to the
            # next file.
            if (currentImg is None):
                print "Error reading image", fileName
                print "The process will read the next image!"
                continue
            # Store the image.
            self.images.append(bridge.cv2_to_imgmsg(currentImg, "bgr8"))

        print len(self.images)

    def accuracyTest(self, imagePath, inputTopic, outputTopic):
        rospy.loginfo("Reading Images")
        self.readImages(imagePath)
        rospy.logdebug("Now test publishing...")
        truePos = len(self.images)
        count = 0
        for image in self.images:
            rospy.loginfo("Sending Image \n")
            self.mockPublish(inputTopic, outputTopic, image)

            if ((self.repliedList[outputTopic]) and
               (len(self.messageList[outputTopic]) == 1)):
                count += 1

        rospy.loginfo("Accuracy %f \n", count/float(truePos))
