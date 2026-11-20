"""Run the serial reader for each selected ForceN tool."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .readers import Readers
from .ros import TOOL_POLL_S, ToolWatch
from .tools import ARMS, DEFAULT_TOOL_MAP, parse_tool_map


class Manager(Node):
    def __init__(self, **kwargs):
        super().__init__('force_n_tool_manager', **kwargs)
        arms = self.declare_parameter('arms', list(ARMS)).value
        entries = self.declare_parameter('tool_map', DEFAULT_TOOL_MAP).value
        gui = self.declare_parameter('gui', True).value
        if (not arms or len(set(arms)) != len(arms) or any(arm not in ARMS for arm in arms)):
            raise ValueError(f'arms must contain distinct names from {", ".join(ARMS)}')
        self.readers = Readers(parse_tool_map(entries), self.get_logger(), gui)
        self.tools = ToolWatch(self, arms)
        self.create_timer(TOOL_POLL_S, self.reconcile)
        self.get_logger().info(
            f'Watching tool_type for {", ".join(arms)}; '
            f'supported tool models: {", ".join(self.readers.tool_map)}')

    def reconcile(self):
        self.readers.reconcile(self.tools.scan())

    def destroy_node(self):
        self.readers.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Manager()
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
