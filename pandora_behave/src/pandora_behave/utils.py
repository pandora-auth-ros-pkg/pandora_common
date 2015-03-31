"""
    Helper functions.

"""

import rospy


def print_debug(debug, msg):
    """ Prints login info using ROS for debuggin purposes. """

    if debug:
        rospy.loginfo(msg)


# TODO Create sender functions for the alerts
#      emulating different distributions

def delay_loop(sender, msg, config):
    """ Sends a msg in a loop followed by a delay

    :param :sender The ROS publisher to send the message.
    :param :msg    The message to be sent.
    :param :config A MockCmd message for the configuration.
    """

    # Rolling back to defaults
    if config.alert_repetitions:
        reps = config.alert_repetitions
    else:
        reps = 1

    # Rolling back to defaults
    if config.alert_delay:
        delay = config.alert_delay
    else:
        delay = 0.5

    if config.debug:
        rospy.loginfo("Sending message...")

    for rep in range(reps):
        if config.debug:
            rospy.loginfo('Sending #' + str(rep + 1) + ' of #' + str(reps) +
                          ' messages.')
            rospy.loginfo(msg)
        sender.publish(msg)
        rospy.sleep(delay)
