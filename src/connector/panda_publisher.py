import rospy
from sensor_msgs.msg import JointState

from config import config


class PandaPublisher(object):
    q_start = config["q_start"]
    topic = config["panda_action_topic"]
    q_dot_stop = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def __init__(self):
        self._pub = rospy.Publisher(self.topic, JointState, queue_size=1)

    def _publish(self, msg):
        self._pub.publish(msg)

    # Publish methods for position/velocity/effort
    def publish_position(self, position):
        msg = JointState()
        msg.name = 7 * ("position",)
        msg.position = position
        self._publish(msg)

    def publish_velocity(self, velocity):
        msg = JointState()
        msg.name = 7 * ("velocity",)
        msg.velocity = velocity
        self._publish(msg)

    def publish_effort(self, effort):
        msg = JointState()
        msg.name = 7 * ("effort",)
        msg.effort = effort
        self._publish(msg)

    # Predefined commands
    def move_to_start(self):
        self.publish_position(self.q_start)

    def stop(self):
        self.publish_velocity(self.q_dot_stop)
