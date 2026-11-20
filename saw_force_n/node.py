import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .driver import Driver
from .tool_types import tool_model, tool_wrench_topic


class ForceNode(Node):
    def __init__(self, args):
        super().__init__('force_n')
        self.declare_parameter('serial_port', args.port)
        self.declare_parameter('baud_rate', args.baud)
        self.declare_parameter('frame_id', args.frame_id)
        self.declare_parameter('tare_timeout', 60.0)
        self.driver = Driver(self.get_parameter('serial_port').value,
                             self.get_parameter('baud_rate').value,
                             self.get_parameter('tare_timeout').value)
        self.publisher = self.create_publisher(WrenchStamped, 'measured_cf', 10)
        self.tool_publisher = None
        if args.tool_model:
            self.tool_publisher = self.create_publisher(WrenchStamped, tool_wrench_topic(args.tool_model), 10)
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
            msg = WrenchStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('frame_id').value
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
            # Torque fields remain zero: this device documents forces only.
            self.publisher.publish(msg)
            if self.tool_publisher is not None:
                self.tool_publisher.publish(msg)

    def publish_status(self):
        state = self.driver.snapshot()['state']
        self.status.publish(String(data=state))
        if state != self.previous_status:
            self.get_logger().info(state)
            self.previous_status = state


def main():
    parser = argparse.ArgumentParser(description='Forcen calibrated force sensor')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--frame-id', default='force_n_sensor')
    parser.add_argument('--tool-model', help='canonical tool ID for the model-specific wrench topic')
    parser.add_argument('--no-gui', action='store_true')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.tool_model is not None and tool_model(args.tool_model) != args.tool_model:
        parser.error('--tool-model must be a canonical model ID such as 400872')
    rclpy.init(args=sys.argv)
    node = ForceNode(args)
    try:
        if args.no_gui:
            rclpy.spin(node)
        else:
            from PyQt5 import QtWidgets, QtCore
            from .gui import Window
            app = QtWidgets.QApplication([sys.argv[0]])
            window = Window(node.driver, namespace=node.get_namespace())
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
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
