#!/usr/bin/env python

import rospy
import message_filters
from connector.msg import State, Action
from sensor_msgs.msg import JointState, TimeReference
from franka_msgs.msg import FrankaState

from panda_publisher import PandaPublisher


class Connector(object):
    def __init__(self, config):
        self.c = config
        self.last_clock = None
        self.use_angular_vel = rospy.get_param("angular_vel", False)

        # Subscribers
        self.clock_sub = message_filters.Subscriber(self.c["clock_topic"],
                                                    TimeReference)
        self.angle_sub = message_filters.Subscriber(self.c["angle_topic"],
                                                    JointState)
        self.panda_sub = message_filters.Subscriber(self.c["panda_state_topic"],
                                                    FrankaState)
        self.action_sub = rospy.Subscriber(self.c["action_topic"], Action,
                                           callback=self.update_action)

        # Publishers
        self.panda_pub = PandaPublisher()
        self.state_pub = rospy.Publisher(self.c["state_topic"], State,
                                         queue_size=1)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.clock_sub, self.angle_sub, self.panda_sub],
            slop=self.c["msg_proximity"],
            queue_size=self.c["message_filter_q_size"])
        self.ts.registerCallback(self.publish_connected_state)

    def publish_connected_state(self, clock, angle, panda):
        if self.c["print_timing_info"]:
            self.print_times(clock, angle, panda)

        # Get header from clock
        current_stamp = clock.header.stamp

        # Read pendulum state
        phi = angle.position[0]
        if self.use_angular_vel:
            dphi = angle.velocity[0]

        # Read robot state
        q = panda.q
        dq = panda.dq

        # Cartesian Coordinates, not used at the moment
        # pose is tuple with 16 entries, column-major
        pose = panda.O_T_EE
        x, y, z = pose[-4:-1]  # Last entry is 1 by convention

        # Create new message
        state_msg = State()
        state_msg.header.stamp = current_stamp
        state_msg.q = q
        state_msg.dq = dq
        state_msg.phi = phi
        if self.use_angular_vel:
            state_msg.dphi = dphi

        self.state_pub.publish(state_msg)

    def update_action(self, action):
        """
        Update class member with latest action, or set to zero action otherwise
        :return:
        """
        stamp = action.header.stamp.to_sec()
        ddq = action.ddq

        if rospy.get_param("debug", False):
            rospy.loginfo("Executed action: " + str(ddq))
        else:
            raise NotImplementedError
            # self.panda_pub.publish_effort(ddq)

    @staticmethod
    def print_times(clock, angle, panda):
        clock_time = clock.header.stamp
        angle_time = angle.header.stamp
        panda_time = panda.header.stamp
        rospy.loginfo("Now  : " + str(rospy.Time.now().to_sec()))
        rospy.loginfo("clock: " + str(clock_time.to_sec()))
        rospy.loginfo("Angle: " + str(angle_time.to_sec()))
        rospy.loginfo("Panda: " + str(panda_time.to_sec()) + '\n')

    def __del__(self):
        self.panda_pub.stop()
