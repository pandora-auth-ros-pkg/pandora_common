"""
    Module to create mock ROS ActionServer.
"""

import rospy

import roslib
roslib.load_manifest('pandora_behave')

from actionlib import SimpleActionServer


class MockActionServer(SimpleActionServer):

    def __init__(self, *args, **kwargs):
        """ Initializing the super class.

        :param :delay Time passed between the request and the response
        :param :success If True the goal will succeed.
        """

        self.delay = kwargs.pop('delay', 3)
        self.success = kwargs.pop('success', True)
        self.preempt = kwargs.pop('success', False)
        self.callback = kwargs.pop('callback', 'simple_delay')

        if self.callback == 'simple_delay':
            kwargs['execute_cb'] = self.simple_delay

        # Removing autostart as a default
        kwargs['auto_start'] = False
        SimpleActionServer.__init__(self, *args, **kwargs)

    def run(self):
        """ Starting up the action server. """

        rospy.init_node(self.name)
        self.start()

    def simple_delay(self, goal):
        """ Responds after the delay. """

        rospy.sleep(self.delay)

        if self.preempt:
            self.set_preempted()

        if self.success:
            self.set_succeeded()
        else:
            self.set_aborted()
