# -*- coding: utf-8 -*-

"""
    Module containing the mock object

    This will be the class to inherit from, if you
    need a more specific mock implementation.

"""
PKG = 'pandora_behave'

import os

import rospy
import roslib
roslib.load_manifest(PKG)

import utils
import config
import msg_factory

from pandora_behave.msg import MockResponse
from pandora_behave.srv import MockTask

from utils import print_debug
from sensor_msgs.msg import Image
from msg_factory import ImageFactory


class Mock(object):
    """ Mock class """

    def __init__(self, name):
        """ Initializes a mock """

        self.name = name
        self.subscribers = {}
        self.publishers = {}
        self.tasks = {}
        self.actor = None
        self.images = []

        self.cmd_topic = '/mock/' + name + '/'
        rospy.init_node(self.name)

        # Register the mock Service
        rospy.Service(self.name + '/set_task', MockTask, self.dispatch)

        # Registering available tasks
        self.register_task('alert', self.alert)
        self.register_task('shutdown', self.shutdown)
        self.register_task('stream images', self.stream_images)

        # Wait for action to take effect
        rospy.sleep(1)

    def register_subscriber(self, topic, msg_type, **kwargs):
        """ Adds a ROS subscriber to this mock.  """

        kwargs.setdefault('callback', None)
        kwargs.setdefault('callback_args', None)
        kwargs.setdefault('queue_size', None)
        kwargs.setdefault('buff_size', 65536)
        kwargs.setdefault('tcp_nodelay', False)

        self.subscribers[topic] = rospy.Subscriber(topic, msg_type, **kwargs)

        # Wait for action to take effect
        rospy.sleep(1)

    def register_publisher(self, topic, msg_type, **kwargs):
        """ Adds a ROS publisher to this mock. """

        kwargs.setdefault('subscriber_listener', None)
        kwargs.setdefault('tcp_nodelay', False)
        kwargs.setdefault('latch', False)
        kwargs.setdefault('headers', None)
        kwargs.setdefault('queue_size', None)

        self.publishers[topic] = rospy.Publisher(topic, msg_type, **kwargs)

        # Wait for action to take effect
        rospy.sleep(1)

    def register_task(self, name, task):
        """ Adds a function to all the available callbacks. """

        self.tasks[name] = task

    def dispatch(self, msg):
        """ The callback for the command interface

        Maps a command to the specified task. When the task is finished
        it notifies the caller with a MockResponse message.

        :param msg: A MockCmd message

        :return A MockResponse message.
        """
        tic = rospy.get_rostime().secs

        response = self.tasks[msg.request.cmd](msg.request)

        tac = rospy.get_rostime().secs

        response.time_taken = tac - tic

        return response

    def alert(self, msg):
        """ Sends an alert of a specified type.

        :param msg: A MockCmd message.

        :return A MockTask message.
        """
        # Creating response message
        response = MockResponse()

        # Declaring what variables from the configuration
        # message will be used.
        destination = msg.input_topic
        alert_type = msg.alert_type
        alert_sender = msg.alert_sender

        if msg.debug:
            rospy.loginfo(self.name + ': Alert')
            rospy.loginfo('-- Type: ' + alert_type)
            rospy.loginfo('-- Topic: ' + destination)

        if not msg.alert_sender:
            rospy.loginfo('Sender not specified. Using default.')
            alert_sender = 'delay_loop'

        # Get the appropriate AlertFactory and AlertType.
        AlertType = getattr(config, alert_type)
        AlertFactory = getattr(msg_factory, alert_type + 'Factory')

        # Create the alert message.
        alert_msg = AlertFactory(msg)

        # Create the publisher.
        self.register_publisher(destination, AlertType)

        # Choosing the way to send the alerts.
        if hasattr(utils, alert_sender):
            Sender = getattr(utils, alert_sender)
        else:
            rospy.logfatal(alert_sender + ' is not supported.')
            exit(1)

        # Sending the alerts.
        Sender(self.publishers[destination], alert_msg, msg)

        response.done = True

        return response

    def stream_images(self, msg):
        """ Reads a series of images from a directory and sends
            them to a specified topic.

        :param source: The directory with the images.
        :param destination: The topic to send the images.
        """
        # Creating response message
        response = MockResponse()

        # Be explicit about what you will use
        # Absolute path directory
        source = msg.directory
        destination = msg.input_topic
        debug = msg.debug
        delay = msg.alert_delay

        rospy.loginfo(delay)

        self.register_publisher(destination, Image)

        self.images = []

        # Use this to log messages
        log_msg = "Reading images from " + source
        print_debug(debug, log_msg)

        if not os.path.isdir(source):
            rospy.logerr("ERROR: " + source + " is not a directory.")
            exit(1)

        for file_name in os.listdir(source):
            current_file = os.path.join(source + '/' + file_name)

            img = ImageFactory(current_file)

            self.images.append(img)

        if not self.images:
            rospy.logerr("ERROR: No image to send...")
            exit(1)
        else:
            log_msg = "INFO: Read " + str(len(self.images)) + " images."
            print_debug(debug, msg)
            log_msg = "INFO: Sending images..."
            print_debug(debug, msg)

            rospy.sleep(1)

            for i, img in enumerate(self.images):
                # This call needs a publisher registered in the 'destination'
                # topic.
                log_msg = "Sending #" + str(i + 1) + " images."
                print_debug(debug, log_msg)
                if img is not None:
                    self.publishers[destination].publish(img)
                    rospy.sleep(delay)

            log_msg = "INFO: I'm done sending images."
            print_debug(debug, log_msg)

        response.done = True

        return response

    def shutdown(self, msg):
        """ Shutdown the mock. """

        # Redundant, but used anyway to avoid
        # the 'unused variable' error from pylint.
        if msg.cmd == 'shutdown':

            for publisher in self.publishers.itervalues():
                publisher.unregister()

            for subscriber in self.subscribers.itervalues():
                subscriber.unregister()

        rospy.signal_shutdown('Mock Command: Shutdown')

        response = MockResponse()
        response.done = True

        return response
