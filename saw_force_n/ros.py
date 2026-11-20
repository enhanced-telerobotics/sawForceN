"""ROS message and tool-selection handling shared by the force nodes."""

from functools import partial
from math import isfinite

from geometry_msgs.msg import WrenchStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .tools import model_id


WRENCH_COLUMNS = ('fx', 'fy', 'fz', 'tx', 'ty', 'tz')
TOOL_POLL_S = 0.2


def force_message(force, frame, stamp):
    msg = WrenchStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = map(float, force)
    # The sensor and the predictor provide force. Torque stays zero.
    return msg


def wrench_features(msg, frame):
    if msg.header.frame_id != frame:
        raise ValueError(f'expected frame {frame!r}, got {msg.header.frame_id!r}')
    force, torque = msg.wrench.force, msg.wrench.torque
    values = (force.x, force.y, force.z, torque.x, torque.y, torque.z)
    if not all(map(isfinite, values)):
        raise ValueError('wrench contains nonfinite values')
    return dict(zip(WRENCH_COLUMNS, values))


class ToolWatch:
    """Read retained dVRK selections and clear them when their publisher is gone."""

    def __init__(self, node, arms, on_change=None):
        self.node = node
        self.models = dict.fromkeys(arms)
        self.topics = {arm: f'/{arm}/tool_type' for arm in arms}
        self.on_change = on_change
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [
            node.create_subscription(String, topic, partial(self.receive, arm), qos)
            for arm, topic in self.topics.items()
        ]

    def receive(self, arm, msg):
        self.update(arm, model_id(msg.data))

    def update(self, arm, model):
        if model != self.models[arm]:
            self.models[arm] = model
            if self.on_change is not None:
                self.on_change(arm, model)

    def scan(self):
        for arm, topic in self.topics.items():
            if self.models[arm] is not None and self.node.count_publishers(topic) == 0:
                self.update(arm, None)
        return self.models
