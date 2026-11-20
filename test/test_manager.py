"""Real ROS subscriptions with simulated reader processes; no hardware access."""

import os
import time
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from saw_force_n.manager import Manager


def test_retained_selection_transfer_conflict_and_publisher_loss(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    context = Context()
    # Isolate the test from the user's dVRK domain. Reader spawning is mocked.
    domain = 100 + os.getpid() % 130
    if domain == int(os.environ.get('ROS_DOMAIN_ID', '0')):
        domain = 99
    context.init(args=[], domain_id=domain)
    executor = SingleThreadedExecutor(context=context)
    source = Node('test_tool_source', context=context)
    manager_node = None
    processes = []

    def spawn(command, **kwargs):
        process = Mock()
        process.command = command
        process.poll.return_value = None
        process.send_signal.side_effect = lambda signal: setattr(process.poll, 'return_value', 0)
        processes.append(process)
        return process

    def until(predicate):
        deadline = time.monotonic() + 6
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert predicate()

    try:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        psm1 = source.create_publisher(String, '/PSM1/tool_type', qos)
        psm2 = source.create_publisher(String, '/PSM2/tool_type', qos)
        psm1.publish(String(data='LND_FORCEN:400906[..]'))
        # Publish before creating the subscriber to exercise retained delivery.
        manager_node = Manager(context=context, parameter_overrides=[Parameter('gui', value=False)])
        manager_node.readers.popen = spawn
        executor.add_node(source)
        executor.add_node(manager_node)
        until(lambda: len(processes) == 1)
        assert processes[0].command[-1] == '__ns:=/PSM1/shaft'
        assert '--no-gui' in processes[0].command

        psm2.publish(String(data='LND_FORCEN:400906[..]'))
        until(lambda: bool(manager_node.readers.conflicts) and not manager_node.readers.readers)
        assert len(processes) == 1
        psm1.publish(String(data=''))
        until(lambda: len(processes) == 2)
        assert processes[1].command[-1] == '__ns:=/PSM2/shaft'

        source.destroy_publisher(psm2)
        until(lambda: not manager_node.readers.readers)
        assert manager_node.tools.models['PSM2'] is None

        # Both Classic MBFs have distinct custom models and separate readers.
        psm2 = source.create_publisher(String, '/PSM2/tool_type', qos)
        psm1.publish(String(data='MBF_FORCEN:400972[..]'))
        psm2.publish(String(data='MBF_FORCEN:400872[..]'))
        until(lambda: set(manager_node.readers.readers) == {'400972', '400872'})
        assert not manager_node.readers.conflicts
        assert {process.command[process.command.index('--port') + 1]
                for process in processes[-2:]} == {'/dev/400972', '/dev/400872'}
    finally:
        if manager_node is not None:
            executor.remove_node(manager_node)
            manager_node.destroy_node()
        executor.remove_node(source)
        source.destroy_node()
        executor.shutdown()
        context.shutdown()
