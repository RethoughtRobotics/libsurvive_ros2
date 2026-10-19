#!/usr/bin/env python3
"""Fit a controller's grip offset (for config/vive_devices.yaml) from a bag.

Record the raw tracker poses while twisting the controller in place about where you hold
it, through all three axes, without moving your hand:

    ros2 bag record -o grip_left /libsurvive/tf

then:

    python3 scripts/fit_grip_offset.py grip_left LHR-337E8983

Least squares for the point c in the controller frame that stays still:
    p_t + R_t c = m   for all samples t.
"""

import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage


def main():
    bag, serial = sys.argv[1], sys.argv[2]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/libsurvive/tf']))
    positions, rotations = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        for tf in deserialize_message(data, TFMessage).transforms:
            if tf.child_frame_id == serial:
                t, q = tf.transform.translation, tf.transform.rotation
                positions.append([t.x, t.y, t.z])
                rotations.append([q.x, q.y, q.z, q.w])
    if len(positions) < 100:
        sys.exit(f'only {len(positions)} poses of {serial} in {bag}')
    p = np.asarray(positions)
    R = Rotation.from_quat(rotations)
    A = np.concatenate([np.hstack([Rm, -np.eye(3)]) for Rm in R.as_matrix()])
    c = np.linalg.lstsq(A, -p.reshape(-1), rcond=None)[0][:3]
    rms = lambda x: np.sqrt(np.mean(np.sum((x - x.mean(axis=0)) ** 2, axis=1)))  # noqa: E731
    span = np.degrees((R[0].inv() * R).magnitude().max())
    print(f'{len(p)} poses, rotated up to {span:.0f} deg')
    print(f'tracking origin moved {rms(p) * 100:.1f} cm rms; grip point moves {rms(p + R.apply(c)) * 100:.1f} cm rms')
    print(f'grip_offset: [{c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}]')


if __name__ == '__main__':
    main()
