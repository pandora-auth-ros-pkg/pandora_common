#!/usr/bin/env python
# encoding: utf-8

import os
import random
import rospy
import rospkg
import actionlib
import genpy
from rosbag import Bag
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import String
from pandora_testing_tools.msg import ReplayBagsAction
from pandora_testing_tools.msg import ReplayBagsGoal
from pandora_testing_tools.srv import SyncAndSplitOrder, SyncAndSplitOrderResponse

'''
flir and kinect should have the same framerate
'''

class SyncAndSplit(object):
    def callback(self, rgb_image, depth_image, flir_image):

        if random.random() > self.testing_percentage:
            # increment counter
            self.train_timesynced += self.timestep
            self.counter += 1
            image_name = self.image_name.format(self.counter)

            self.trainbag.write(self.rgb_topic, rgb_image, genpy.Time(self.train_timesynced))
            self.trainbag.write(self.depth_topic, depth_image, genpy.Time(self.train_timesynced))
            self.trainbag.write(self.flir_topic, flir_image, genpy.Time(self.train_timesynced))
            self.trainbag.write("/image_frame_name", String(image_name), genpy.Time(self.train_timesynced))

            # write lossless representation of the images in the subdirectories with
            # the same name as the images string
            self.convert_msg_to_image(self.rgb_train_dir + image_name, rgb_image, 'rgb8')
            self.convert_msg_to_image(self.depth_train_dir + image_name, depth_image, 'mono8')
            self.convert_array_to_image(self.thermal_train_dir + image_name, flir_image, rgb_image.header.stamp)

        else:
            self.test_timesynced += self.timestep
            self.testbag.write(self.rgb_topic, rgb_image, genpy.Time(self.test_timesynced))
            self.testbag.write(self.depth_topic, depth_image, genpy.Time(self.test_timesynced))
            self.testbag.write(self.flir_topic, flir_image, genpy.Time(self.test_timesynced))


    def __init__(self, testbag_name='test.bag', trainbag_name='train.bag',
            testing_percentage=0.2, start_counter=0):

        random.seed()
        self.testing_percentage = testing_percentage
        self.counter = start_counter
        self.testbag_name = testbag_name
        self.trainbag_name = trainbag_name

        self.test_timesynced = 0.01
        self.train_timesynced = 0.01
        self.timestep = 0.2

        self.rgb_topic = '/kinect/rgb/image_raw'
        self.depth_topic = '/kinect/depth/image_raw'
        self.flir_topic = '/flir_raspberry/image'

        self.image_name = "/frame_{0}.png"

        self.bridge = CvBridge()

        self.init_syncronizer()

        self.server = rospy.Service('order', SyncAndSplitOrder, self.service_callback)

    def setUp(self, dataset_dir):

        # check for subdirectories and create them
        self.dataset_dir = dataset_dir
        self.testing_dataset_dir = self.dataset_dir + "/testing"
        self.training_dataset_dir = self.dataset_dir + "/training"
        self.rgb_train_dir = self.training_dataset_dir + "/rgb"
        self.depth_train_dir = self.training_dataset_dir + "/depth"
        self.thermal_train_dir = self.training_dataset_dir + "/thermal"
        if not os.path.isdir(self.testing_dataset_dir):
            os.makedirs(self.testing_dataset_dir)
        if not os.path.isdir(self.training_dataset_dir):
            os.makedirs(self.training_dataset_dir)
        if not os.path.isdir(self.rgb_train_dir):
            os.makedirs(self.rgb_train_dir)
        if not os.path.isdir(self.depth_train_dir):
            os.makedirs(self.depth_train_dir)
        if not os.path.isdir(self.thermal_train_dir):
            os.makedirs(self.thermal_train_dir)

        self.init_rosbag("/test/bag_player",
                self.testing_dataset_dir + '/' + self.testbag_name,
                self.training_dataset_dir + '/' + self.trainbag_name)


    def init_syncronizer(self):

        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.flir_sub = message_filters.Subscriber(self.flir_topic, UInt8MultiArray)

        self.syncronizer = message_filters.TimeSynchronizer(
                [self.rgb_sub, self.depth_sub, self.flir_sub], 10)
        self.syncronizer.registerCallback(self.callback)

    def init_rosbag(self, inbag_topic, testbag_name, trainbag_name):

        self.testbag = Bag(testbag_name, 'w')
        self.trainbag = Bag(trainbag_name, 'w')
        self.inbag_client = actionlib.SimpleActionClient(inbag_topic, ReplayBagsAction)
        self.inbag_client.wait_for_server()
        self.inbag_goal = ReplayBagsGoal()
        self.inbag_goal.start = True

    def service_callback(self, req):

        self.setUp(req.dataset_dir)
        self.start()
        return SyncAndSplitOrderResponse(True)

    def start(self):

        self.inbag_client.send_goal(self.inbag_goal)
        self.inbag_client.wait_for_result()
        # close outbag files
        self.testbag.close()
        self.trainbag.close()

    def convert_msg_to_image(self, filename, msg, encoding='rgb8'):
        ''' mby default encoding should be bgr8 '''
        image = self.bridge.imgmsg_to_cv2(msg, encoding)
        cv2.imwrite(filename, image, [cv2.IMWRITE_PNG_COMPRESSION, 0])

    def convert_array_to_image(self, filename, array, stamp):

        msg = Image()
        msg.data = array.data
        msg.header.frame_id = "/flir_optical_frame"
        msg.header.stamp = stamp
        # for info in array.layout.dim:
        #     if info.label == "height":
        #         msg.height = info.size
        #     if info.label == "width":
        #         msg.width = info.size
        msg.height = array.layout.dim[1].size
        msg.width = array.layout.dim[0].size
        msg.encoding = 'mono8'
        msg.step = 1

        self.convert_msg_to_image(filename, msg, msg.encoding)


if __name__ == '__main__':
    from sys import argv
    rospy.init_node("sync_and_split", argv, log_level=rospy.INFO)
    sync_and_split = SyncAndSplit()
    rospy.spin()
