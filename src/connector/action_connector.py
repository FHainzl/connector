#!/usr/bin/env python

import rospy

from connector.msg import Action
from sensor_msgs.msg import TimeReference

# from panda_utilities.panda_publisher import PandaPublisher
from panda_utilities import PandaPublisher

class ActionConnector(object):
    def __init__(self, config):
        self.c = config
        self.current_action = None
        self.current_stamp = None

        # Subscribers
        self.clock_sub = rospy.Subscriber(self.c["clock_topic"], TimeReference,
                                          callback=self.execute_action)
        self.action_sub = rospy.Subscriber(self.c["action_topic"], Action,
                                           callback=self.update_action)

        # Publishers
        self.panda_pub = PandaPublisher()

    def update_action(self, action):
        """
        Update class member with latest action, or set to zero action otherwise
        :return:
        """
        stamp = action.header.stamp.to_sec()
        ddq = action.ddq

        self.current_action = ddq
        self.current_stamp = stamp

    def execute_action(self, time):
        """
        At the tick of the clock, execute action, if it's a current one
        """
        # Nothing to publish yet
        if self.current_stamp is None:
            return

        debug = rospy.get_param("debug", False)

        now = rospy.Time.now().to_sec()

        dt = self.c["clock_freq"] ** -1
        # Execute action if it is based on state from last clock, else stop
        if now - self.current_stamp < 1.5 * dt:
            if not debug:
                self.panda_pub.publish_effort(self.current_action)
            else:
                rospy.loginfo("To be executed: " + str(self.current_action))
        else:
            self.panda_pub.stop()

    def __del__(self):
        self.panda_pub.stop()
