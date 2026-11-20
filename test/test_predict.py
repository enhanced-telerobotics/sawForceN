"""Exercise the inference topics with a checkpoint and synthetic ROS inputs."""

import os
import time

import numpy as np
import pytest

torch = pytest.importorskip('torch')
pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from saw_force_n.models import ForcePredictor
from saw_force_n.predict import Predictor


def save_checkpoint(path, model='400872', gain=1.):
    torch.save({
        'format_version': 1,
        'metadata': {
            'model_config': {'model_type': 'ridge'},
            'tool_type': f'MBF_FORCEN:{model}[..]',
            'input_columns': ['jaw_effort', 'fx', 'roll_position'],
            'target_frame': 'shaft',
            'target_units': 'N',
            'sequence_length': 1,
            'sample_period_s': 0.01,
        },
        'state_dict': {
            'input_mean': torch.zeros(3),
            'input_std': torch.ones(3),
            'target_mean': torch.zeros(3),
            'target_std': torch.ones(3),
            'affine_weights': torch.tensor([
                [0., 0., gain], [gain, 0., 0.], [0., gain, 0.], [0., 0., 0.],
            ]),
        },
    }, path)


def test_checkpoint_rejects_wrong_tool(tmp_path):
    path = tmp_path / '400972.pt'
    save_checkpoint(path)
    with pytest.raises(ValueError, match='does not match 400972'):
        ForcePredictor(path, '400972')


def test_shaft_and_ecm_output_for_active_tool(tmp_path, monkeypatch):
    save_checkpoint(tmp_path / '400872.pt')
    save_checkpoint(tmp_path / '400972.pt', '400972', gain=2.)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    domain = 100 + os.getpid() % 130
    if domain == int(os.environ.get('ROS_DOMAIN_ID', '0')):
        domain = 99
    context = Context()
    context.init(args=[], domain_id=domain)
    executor = SingleThreadedExecutor(context=context)
    source = Node('test_force_source', context=context)
    predictor = None
    shaft, ecm = [], []
    try:
        tool = source.create_publisher(
            String, '/PSM2/tool_type', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        sensor = source.create_publisher(WrenchStamped, '/PSM2/shaft/tool_400872/measured_cf', 10)
        next_sensor = source.create_publisher(WrenchStamped, '/PSM2/shaft/tool_400972/measured_cf', 10)
        arm = source.create_publisher(JointState, '/PSM2/measured_js', 10)
        jaw = source.create_publisher(JointState, '/PSM2/jaw/measured_js', 10)
        global_pose = source.create_publisher(PoseStamped, '/PSM2/measured_cp', 10)
        local_pose = source.create_publisher(PoseStamped, '/PSM2/local/measured_cp', 10)
        source.create_subscription(WrenchStamped, '/PSM2/shaft/predicted_cf', shaft.append, 10)
        source.create_subscription(WrenchStamped, '/PSM2/measured_cf', ecm.append, 10)
        tool.publish(String(data='MBF_FORCEN:400872[..]'))
        predictor = Predictor('PSM2', tmp_path, context=context)
        executor.add_node(source)
        executor.add_node(predictor)

        def spin_until(predicate, seconds=4):
            deadline = time.monotonic() + seconds
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            assert predicate()

        def publish_samples(global_frame='ECM', *, reader=sensor):
            stamp = source.get_clock().now().to_msg()
            wrench = WrenchStamped()
            wrench.header.stamp = stamp
            wrench.header.frame_id = 'force_n_sensor'
            wrench.wrench.force.x = 2.
            joints = JointState(name=['roll', 'yaw', 'pitch'], position=[0., -1.5708, 1.5708])
            joints.header.stamp = stamp
            arm.publish(joints)
            jaw_sample = JointState(name=['jaw'], effort=[3.])
            jaw_sample.header.stamp = stamp
            jaw.publish(jaw_sample)
            local = PoseStamped()
            local.header.stamp = stamp
            local.header.frame_id = 'PSM2_base'
            local.pose.orientation.w = 1.
            local_pose.publish(local)
            registered = PoseStamped()
            registered.header.stamp = stamp
            registered.header.frame_id = global_frame
            registered.pose.orientation.z = registered.pose.orientation.w = 2 ** -0.5
            global_pose.publish(registered)
            def inputs_received():
                keys = ('arm', 'shaft_joints', 'jaw', 'base_pose')
                ready = all(key in predictor.samples and predictor.samples[key].msg.header.stamp == stamp
                            for key in keys)
                if global_frame == 'ECM':
                    pose = predictor.samples.get('ecm_pose')
                    return ready and pose is not None and pose.msg.header.stamp == stamp
                return ready and 'ecm_pose' not in predictor.samples

            spin_until(inputs_received)
            reader.publish(wrench)
            return stamp

        spin_until(lambda: predictor.tools.models['PSM2'] == '400872' and
                   all(pub.get_subscription_count() for pub in (sensor, arm, jaw, global_pose, local_pose)) and
                   predictor.out_shaft.get_subscription_count() and predictor.out_ecm.get_subscription_count())
        stamp = publish_samples()
        spin_until(lambda: shaft and ecm)
        s, c = shaft[-1], ecm[-1]
        assert s.header.frame_id == 'force_n_sensor'
        assert c.header.frame_id == 'ECM'
        assert s.header.stamp == c.header.stamp == stamp
        np.testing.assert_allclose([s.wrench.force.x, s.wrench.force.y, s.wrench.force.z], [2, 0, 3])
        np.testing.assert_allclose([c.wrench.force.x, c.wrench.force.y, c.wrench.force.z], [3, 2, 0], atol=2e-5)
        assert c.wrench.torque.x == c.wrench.torque.y == c.wrench.torque.z == 0.

        publish_samples(global_frame='PSM2_base')
        spin_until(lambda: len(shaft) == 2)
        assert len(ecm) == 1

        tool.publish(String(data='MBF_FORCEN:400972[..]'))
        spin_until(lambda: predictor.tools.models['PSM2'] == '400972' and next_sensor.get_subscription_count())
        publish_samples()
        deadline = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert len(shaft) == 2 and len(ecm) == 1

        stamp = publish_samples(reader=next_sensor)
        spin_until(lambda: len(shaft) == 3 and len(ecm) == 2)
        assert shaft[-1].header.stamp == ecm[-1].header.stamp == stamp
        assert shaft[-1].wrench.force.x == 4.

        source.destroy_publisher(tool)
        spin_until(lambda: predictor.model is None and predictor.tools.models['PSM2'] is None)
    finally:
        if predictor is not None:
            executor.remove_node(predictor)
            predictor.destroy_node()
        executor.remove_node(source)
        source.destroy_node()
        executor.shutdown()
        context.shutdown()
