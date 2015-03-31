# -*- coding: utf-8 -*-


"""
    Module for the Reporter object.

    The Reporter is collecting data from the output topic
    and given a set of rules and goals calculates the performance
    metrics.
"""


import rospy

import config


class Reporter(object):
    """ Reporter class """

    def __init__(self, topic, msg_type, debug=False):
        """ Initialize """

        self.topic = topic
        self.msg_type = msg_type
        self.data = []
        self.debug = debug
        if hasattr(config, msg_type):
            self.alert_type = getattr(config, msg_type)
        else:
            rospy.logfatal(msg_type + ' is not supported by the Reporter.')
            exit(1)

        self.subscriber = rospy.Subscriber(topic, self.alert_type, self.collect)
        rospy.sleep(1)

    def simple_accuracy(self, total):
        """ A simple accuracy metric counting the received messages.

        :param total: The number of alerts for the best case scenario
        """

        return len(self.data) / float(total)

    def buffer_size(self):
        """ Returns the number of messages received from this reporter. """

        return len(self.data)

    def is_empty(self):
        """ Returns True if the buffer is empty """

        return len(self.data) == 0

    def echo(self):
        """ Echo the input. """

        for msg in self.data:
            rospy.loginfo(msg)

    def collect(self, msg):
        """ The main function for all the incoming messages.

        :param msg: The message send into the topic.
        """
        if self.debug:
            rospy.logdebug(self.topic + " : Message received.")
        self.data.append(msg)

    def shutdown(self):
        """ Shutdowns the reporter. """

        self.subscriber.unregister()
