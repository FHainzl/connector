#!/usr/bin/env python


import rospy
from sensor_msgs.msg import TimeReference


class Clock(object):
    def __init__(self, config):
        self.c = config
        self.freq = self.c["clock_freq"]
        self.topic = self.c["clock_topic"]

        self.pub = rospy.Publisher(self.topic, TimeReference, queue_size=1)

    def run(self):
        r = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            t = TimeReference()
            t.source = str(self.freq)
            t.header.stamp = rospy.Time.now()
            if rospy.get_param("RL_agent_active", True):
                self.pub.publish(t)
            r.sleep()
