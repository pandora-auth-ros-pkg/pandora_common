# -*- coding: utf-8 -*-


"""
    Module for the Coordinator object.

    Coordinator is responsible for executing a given
    test scenario involving more than one node. It gives commands
    to the available Mocks and uses a Reporter to decide the outcome
    of the test.

"""
PKG = 'pandora_behave'

import rospy
import roslib

roslib.load_manifest(PKG)

from state_manager.state_client import StateClient
from pandora_behave.msg import MockCmd, MockResponse
from pandora_behave.srv import MockTask
from reporter import Reporter


class Coordinator(object):
    """ Coordinator class """

    def __init__(self, name, with_state_client=True):
        """ Initialize """

        self.name = name
        self.mocks = {}
        self.reporters = {}

        if with_state_client:
            self.state_changer = StateClient()
            self.state_changer.client_initialize()

        rospy.init_node(self.name)

    def set_state(self, state):
        """ Changes the state of the StateManager """

        if self.state_changer:
            self.state_changer.transition_to_state(state)
            rospy.sleep(0.5)
        else:
            rospy.logerr("A state client hasn't been initialized")

    def register_mock(self, name):
        """ Creates a publisher responsible for sending commands to
            the mock.
        """

        self.mocks[name] = rospy.Publisher('/mock/' + name + '/', MockCmd)

        # Wait for action to take effect
        rospy.sleep(1)

    def register_reporter(self, name, topic, msg_type):
        """ Creates a reporter

        :param name: The name of the reporter.
        :param topic: A topic to listen to.
        :param msg_type: The type of messages it will receive.
        """

        self.reporters[name] = Reporter(topic, msg_type)

    def reporter(self, name):
        """ Returns a Reporter object

        :param name: The name of the reporter
        """
        if self.reporters[name]:
            return self.reporters[name]
        else:
            rospy.logerr(name + ' is not a registered reporter')

    def send_command(self, mock, msg):
        """ Sends a command to a mock service

        :param :mock The name of the mock
        :param :msg A MockCmd message
        """
        service_name = mock + '/set_task'
        rospy.wait_for_service(service_name)
        try:
            task = rospy.ServiceProxy(service_name, MockTask)
            return task(msg)
        except rospy.ServiceException, e:
            rospy.logerr('Failed call @' + service_name + ' :' + str(e))

    def shutdown(self, reason, with_mocks=False):
        """ Shutdowns the coordinator and all the related objects. """
        response = MockResponse()

        for reporter in self.reporters.itervalues():
            reporter.shutdown()

        if with_mocks:
            mocks_responses = 0

            # Creating shutdown message
            msg = MockCmd()
            msg.cmd = 'shutdown'

            for mock in self.mocks.iterkeys():
                res = self.send_command(mock, msg)
                if res.done:
                    mocks_responses += 1
            if mocks_responses == len(self.mocks):
                response.done = True

        return response
