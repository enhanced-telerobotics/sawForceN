"""Watch dVRK tool selections and manage ForceN readers throughout the session."""

import re

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .tool_manager import DEFAULT_TOOL_MAP, ToolManager, parse_tool_map


class ToolManagerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('force_n_tool_manager', **kwargs)
        arms = self.declare_parameter('arms', ['PSM1', 'PSM2', 'PSM3']).value
        entries = self.declare_parameter('tool_map', DEFAULT_TOOL_MAP).value
        gui = self.declare_parameter('gui', True).value
        if (not arms or len(set(arms)) != len(arms)
                or any(not re.fullmatch(r'PSM[1-3]', arm) for arm in arms)):
            raise ValueError('arms must contain distinct names from PSM1, PSM2, PSM3')
        self.manager = ToolManager(arms, parse_tool_map(entries), self.get_logger(), gui)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tool_subscriptions = [
            self.create_subscription(
                String, f'/{arm}/tool_type',
                lambda msg, arm=arm: self.manager.update(arm, msg.data), qos)
            for arm in arms
        ]
        self.create_timer(0.2, self.check_tools)
        self.get_logger().info(
            f'Watching tool_type for {", ".join(arms)}; '
            f'supported tool models: {", ".join(self.manager.tool_map)}')

    def check_tools(self):
        for arm in self.manager.tool_types:
            # tool_type is an event, not a heartbeat. Drop retained state only
            # when its publisher disappears from the ROS graph.
            if self.count_publishers(f'/{arm}/tool_type') == 0:
                self.manager.update(arm, '')
        self.manager.reconcile()

    def destroy_node(self):
        self.manager.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = ToolManagerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
