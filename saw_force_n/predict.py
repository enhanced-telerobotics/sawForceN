"""Select a model from dVRK tool events and publish corrected ForceN force."""

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped, WrenchStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

from .frames import (ARM_COLUMNS, JAW_COLUMNS, SHAFT_JOINTS, force_in_ecm,
                     joint_features, quaternion_rotation)
from .ros import TOOL_POLL_S, WRENCH_COLUMNS, ToolWatch, force_message, wrench_features
from .tools import ARMS, MEASURED_CF, SHAFT_FRAME, shaft_namespace, tool_wrench_topic


@dataclass
class Sample:
    msg: object
    value: object
    received: float
    stamp_ns: int


def model_directories(directory):
    config = Path(os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config') / 'force_n' / 'models'
    ros = Path(os.environ.get('ROS_HOME') or Path.home() / '.ros') / 'force_n' / 'models'
    requested = Path(directory).expanduser() if directory else config
    return (config, ros) if requested == config else (requested,)


class Predictor(Node):
    def __init__(self, arm, model_directory='', **kwargs):
        super().__init__('force_n_predictor', namespace=shaft_namespace(arm), **kwargs)
        self.arm = arm
        self.model_dirs = model_directories(model_directory)
        self.sensor_frame = self.declare_parameter('sensor_frame', SHAFT_FRAME).value
        self.base_frame = self.declare_parameter('psm_base_frame', f'{arm}_base').value
        self.ecm_frame = self.declare_parameter('ecm_frame', 'ECM').value
        self.stale_timeout = self.declare_parameter('stale_timeout_s', 0.1).value
        self.max_skew = self.declare_parameter('max_source_skew_s', 0.05).value
        if not all(np.isfinite(value) and value > 0 for value in (self.stale_timeout, self.max_skew)):
            raise ValueError('Input timeouts must be positive and finite')

        self.model = None
        self.sensor_sub = None
        self.inference_timer = None
        self.samples = {}
        self.last_sensor = None
        self.out_shaft = self.create_publisher(WrenchStamped, 'predicted_cf', 10)
        self.out_ecm = self.create_publisher(WrenchStamped, f'/{arm}/{MEASURED_CF}', 10)

        self.tools = ToolWatch(self, (arm,), self.select_model)
        self.create_timer(TOOL_POLL_S, self.tools.scan)
        self.create_subscription(JointState, f'/{arm}/measured_js', self.on_arm, qos_profile_sensor_data)
        self.create_subscription(JointState, f'/{arm}/jaw/measured_js',
                                 self.on_jaw, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f'/{arm}/measured_cp',
                                 self.on_ecm_pose, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f'/{arm}/local/measured_cp',
                                 self.on_base_pose, qos_profile_sensor_data)

    def select_model(self, _arm, model):
        self.model = None
        self.samples.clear()
        self.last_sensor = None
        if self.sensor_sub is not None:
            self.destroy_subscription(self.sensor_sub)
            self.sensor_sub = None
        if self.inference_timer is not None:
            self.destroy_timer(self.inference_timer)
            self.inference_timer = None
        if model is None:
            return

        filename = f'{model}.pt'
        try:
            checkpoint = next((directory / filename for directory in self.model_dirs
                               if (directory / filename).is_file()), None)
            if checkpoint is None:
                self.get_logger().info(f'No checkpoint for {model} in {", ".join(map(str, self.model_dirs))}')
                return
            import torch

            from .models import ForcePredictor

            torch.set_num_threads(1)
            predictor = ForcePredictor(checkpoint, model)
            columns = predictor.input_columns
            unsupported = set(columns) - set(ARM_COLUMNS + JAW_COLUMNS + WRENCH_COLUMNS)
            if unsupported:
                raise ValueError(f'Unsupported model inputs: {sorted(unsupported)}')
            roll_R_sensor = quaternion_rotation(
                predictor.metadata.get('roll_R_sensor_xyzw', [0., 0., 0., 1.]),
            )
        except Exception as exc:
            self.get_logger().warning(f'Cannot load {filename}: {exc}')
            return

        self.model = predictor
        self.roll_R_sensor = roll_R_sensor
        self.arm_columns = tuple(c for c in columns if c in ARM_COLUMNS)
        self.jaw_columns = tuple(c for c in columns if c in JAW_COLUMNS)
        self.model_sources = ['sensor']
        if self.arm_columns:
            self.model_sources.append('arm')
        if self.jaw_columns:
            self.model_sources.append('jaw')
        # Raw readers can still be stopping when dVRK reports a different tool.
        # A model-specific topic keeps the outgoing sensor out of the new model.
        self.sensor_sub = self.create_subscription(
            WrenchStamped, tool_wrench_topic(model),
            lambda msg: self.on_sensor(msg, model), qos_profile_sensor_data,
        )
        # The driver may publish faster than training. Consume each fresh sample
        # at most once, on the checkpoint-period ticks.
        self.inference_timer = self.create_timer(predictor.sample_period_s, self.tick)
        self.get_logger().info(f'Loaded {model} checkpoint {checkpoint} for {self.arm}')

    def cache(self, key, msg, read):
        if self.model is None:
            return
        stamp = msg.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        previous = self.samples.get(key)
        try:
            if ns:
                age = (self.get_clock().now().nanoseconds - ns) / 1e9
                if not -self.max_skew <= age <= self.stale_timeout:
                    raise ValueError('source timestamp is stale or in the future')
                if previous and previous.stamp_ns:
                    if ns == previous.stamp_ns:
                        return
                    if ns < previous.stamp_ns:
                        raise ValueError('source timestamp moved backwards')
            self.samples[key] = Sample(msg, read(), time.monotonic(), ns)
        except ValueError as exc:
            self.samples.pop(key, None)
            if key in self.model_sources:
                self.model.reset()
            self.get_logger().warning(f'Invalid {key}: {exc}', throttle_duration_sec=5.0)

    def on_sensor(self, msg, model):
        if model == self.tools.models[self.arm]:
            self.cache('sensor', msg, lambda: wrench_features(msg, self.sensor_frame))

    def on_arm(self, msg):
        if self.model is None:
            return
        if self.arm_columns:
            self.cache('arm', msg, lambda: joint_features(msg, self.arm_columns))
        self.cache('shaft_joints', msg, lambda: joint_features(msg, SHAFT_JOINTS))

    def on_jaw(self, msg):
        if self.model is not None and self.jaw_columns:
            self.cache('jaw', msg, lambda: joint_features(msg, self.jaw_columns))

    def on_pose(self, key, frame, msg):
        def read():
            if msg.header.frame_id != frame:
                raise ValueError(f'expected frame {frame!r}, got {msg.header.frame_id!r}')
            q = msg.pose.orientation
            return quaternion_rotation((q.x, q.y, q.z, q.w))

        self.cache(key, msg, read)

    def on_ecm_pose(self, msg):
        self.on_pose('ecm_pose', self.ecm_frame, msg)

    def on_base_pose(self, msg):
        self.on_pose('base_pose', self.base_frame, msg)

    def fresh(self, keys, now, now_ns):
        samples = [self.samples.get(key) for key in keys]
        if any(sample is None or now - sample.received > self.stale_timeout for sample in samples):
            return False
        if any(sample.stamp_ns and
               not -self.max_skew <= (now_ns - sample.stamp_ns) / 1e9 <= self.stale_timeout
               for sample in samples):
            return False
        stamps = [sample.stamp_ns for sample in samples if sample.stamp_ns]
        receipts = [sample.received for sample in samples]
        return ((len(stamps) < 2 or (max(stamps) - min(stamps)) / 1e9 <= self.max_skew)
                and (len(stamps) == len(samples) or max(receipts) - min(receipts) <= self.max_skew))

    def tick(self):
        now = time.monotonic()
        ros_now = self.get_clock().now()
        if self.model is None:
            return
        if not self.fresh(self.model_sources, now, ros_now.nanoseconds):
            self.model.reset()
            return
        sensor = self.samples['sensor']
        key = (sensor.stamp_ns, 0.0 if sensor.stamp_ns else sensor.received)
        if key == self.last_sensor:
            return
        self.last_sensor = key
        features = {}
        for source in self.model_sources:
            features.update(self.samples[source].value)
        try:
            prediction = self.model.predict(
                [features[column] for column in self.model.input_columns],
                sensor.stamp_ns / 1e9 if sensor.stamp_ns else sensor.received,
            )
        except (ValueError, RuntimeError) as exc:
            self.model.reset()
            self.get_logger().warning(f'Prediction rejected: {exc}', throttle_duration_sec=5.0)
            return
        if prediction is None:
            return

        stamp = sensor.msg.header.stamp if sensor.stamp_ns else ros_now.to_msg()
        self.out_shaft.publish(force_message(prediction, self.sensor_frame, stamp))
        frame_sources = [*self.model_sources, 'shaft_joints', 'ecm_pose', 'base_pose']
        if self.fresh(frame_sources, now, ros_now.nanoseconds):
            ecm_force = force_in_ecm(
                prediction, self.samples['shaft_joints'].value, self.roll_R_sensor,
                self.samples['ecm_pose'].value, self.samples['base_pose'].value,
            )
            self.out_ecm.publish(force_message(ecm_force, self.ecm_frame, stamp))


def main():
    parser = argparse.ArgumentParser(description='Publish corrected shaft and ECM force')
    parser.add_argument('--arm', required=True, choices=ARMS)
    parser.add_argument('--model-directory', default='')
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = None
    try:
        node = Predictor(args.arm, args.model_directory)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        get_logger('force_n_predictor').error(f'{args.arm} prediction stopped: {exc}')
        raise SystemExit(1) from None
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
