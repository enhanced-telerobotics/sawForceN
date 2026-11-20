"""Publish calibrated ForceN samples and handle hardware tare requests."""

import argparse
import os
import signal
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .driver import Driver
from .ports import default_port
from .ros import force_message
from .tools import MEASURED_CF, SHAFT_FRAME, model_id, sensor_node_name


class Sensor(Node):
    def __init__(self, args, **kwargs):
        super().__init__(sensor_node_name(args.tool_model), **kwargs)
        self.declare_parameter('serial_port', args.port)
        self.declare_parameter('baud_rate', args.baud)
        self.declare_parameter('frame_id', args.frame_id)
        self.declare_parameter('tare_timeout', 60.0)
        self.driver = Driver(self.get_parameter('serial_port').value,
                             self.get_parameter('baud_rate').value,
                             self.get_parameter('tare_timeout').value)
        self.publisher = self.create_publisher(WrenchStamped, MEASURED_CF, 10)
        self.status = self.create_publisher(String, 'status', 10)
        self.create_service(Trigger, 'rebias', self.rebias)
        self.create_timer(.005, self.publish_samples)
        self.create_timer(.2, self.publish_status)
        self.previous_status = None
        self.driver.thread.start()

    def rebias(self, request, response):
        response.success = self.driver.request_tare()
        response.message = ('Hardware tare accepted; monitor status for Streaming or Error'
                            if response.success else 'Sensor not streaming or tare already pending')
        return response

    def publish_samples(self):
        for force in self.driver.take_samples():
            msg = force_message(force, self.get_parameter('frame_id').value, self.get_clock().now().to_msg())
            self.publisher.publish(msg)

    def publish_status(self):
        state = self.driver.snapshot()['state']
        self.status.publish(String(data=state))
        if state != self.previous_status:
            self.get_logger().info(state)
            self.previous_status = state


def run_gui(node, model):
    from PyQt5 import QtCore, QtWidgets

    from .gui import Window

    app = QtWidgets.QApplication([sys.argv[0]])
    window = Window(node.driver, namespace=node.get_namespace(), model=model)
    timer = QtCore.QTimer()

    def tick():
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
        else:
            app.quit()

    timer.timeout.connect(tick)
    timer.start(5)
    window.show()
    app.exec_()


def main():
    parser = argparse.ArgumentParser(description='Forcen calibrated force sensor')
    parser.add_argument('-p', '--port', default=default_port())
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--frame-id', default=SHAFT_FRAME)
    parser.add_argument('--tool-model', help='canonical tool ID for the managed sensor identity')
    parser.add_argument('--no-gui', action='store_true')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.tool_model is not None and model_id(args.tool_model) != args.tool_model:
        parser.error('--tool-model must be a canonical model ID such as 400872')
    if os.name == 'nt':
        # The manager sends Ctrl+Break to this reader's process group.
        signal.signal(signal.SIGBREAK, signal.default_int_handler)
    rclpy.init(args=sys.argv)
    node = None
    try:
        node = Sensor(args)
        if args.no_gui:
            rclpy.spin(node)
        else:
            run_gui(node, args.tool_model)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.driver.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
