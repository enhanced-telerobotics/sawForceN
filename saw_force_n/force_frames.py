"""ForceN feature ordering and frame rotations used by the training extractor."""

import numpy as np

ARM_JOINTS = ('yaw', 'pitch', 'insertion', 'roll', 'wrist_pitch', 'wrist_yaw')
SHAFT_JOINTS = ('yaw_position', 'pitch_position', 'roll_position')
JOINT_FIELDS = ('position', 'velocity', 'effort')
WRENCH_COLUMNS = ('fx', 'fy', 'fz', 'tx', 'ty', 'tz')
ARM_COLUMNS = tuple(f'{name}_{field}' for name in ARM_JOINTS for field in JOINT_FIELDS)
JAW_COLUMNS = tuple(f'jaw_{field}' for field in JOINT_FIELDS)


def joint_features(msg, columns):
    if len(set(msg.name)) != len(msg.name):
        raise ValueError('JointState contains duplicate joint names')
    indices = {name: i for i, name in enumerate(msg.name)}
    features = {}
    for column in columns:
        name, field = column.rsplit('_', 1)
        values = getattr(msg, field)
        if name not in indices or len(values) != len(msg.name):
            raise ValueError(f'Missing JointState feature {column}')
        features[column] = values[indices[name]]
    if not np.isfinite(list(features.values())).all():
        raise ValueError('JointState contains a nonfinite feature')
    return features


def quaternion_rotation(xyzw):
    q = np.asarray(xyzw, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError('Expected a finite xyzw quaternion')
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid quaternion norm')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ])


def _rx(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def _rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def psm_base_R_shaft(joints):
    """First four modified DH links from the ForceN extraction kinematics."""
    yaw, pitch, roll = (joints[name] for name in SHAFT_JOINTS)
    # Insertion contributes only translation. Use the dVRK JSON's decimal angles.
    return (_rx(1.5708) @ _rz(yaw + 1.5708) @ _rx(-1.5708)
            @ _rz(pitch - 1.5708) @ _rx(1.5708) @ _rz(roll))


def force_in_ecm(force, joints, roll_R_sensor, ecm_R_tip, base_R_tip):
    # The local and registered poses describe the same tip. Its wrist and
    # tooltip rotations cancel, yielding the ECM-to-PSM-base registration.
    ecm_R_base = ecm_R_tip @ base_R_tip.T
    return ecm_R_base @ psm_base_R_shaft(joints) @ roll_R_sensor @ force
