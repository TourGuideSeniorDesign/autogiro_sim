#!/usr/bin/env python3
"""Drop PointCloud2 returns that fall inside the robot's own bounding box.

The lidar is mounted off-center, so a radial range_min in
pointcloud_to_laserscan can't distinguish a real return 0.3 m behind the
robot from a self-hit on the chassis 0.3 m in front. This node transforms
each cloud into base_link and removes any point inside an axis-aligned box
covering the robot body.
"""

import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener


class CropSelfHits(Node):
    def __init__(self):
        super().__init__('crop_self_hits')
        p = self.declare_parameters('', [
            ('target_frame', 'base_link'),
            ('min_x', -0.05), ('max_x', 0.98),
            ('min_y', -0.42), ('max_y', 0.42),
            ('min_z', -0.10), ('max_z', 1.90),
            ('input_topic', '/points/points'),
            ('output_topic', '/points_filtered'),
        ])
        self.target_frame = self.get_parameter('target_frame').value
        self.bbox = tuple(self.get_parameter(k).value for k in (
            'min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value,
            qos_profile_sensor_data,
        )
        self.sub = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value,
            self.cb, qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Cropping self-hits in {self.target_frame} bbox={self.bbox}')

    def cb(self, msg: PointCloud2):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        struct = pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.column_stack(
            (struct['x'], struct['y'], struct['z'])).astype(np.float32)
        pts = pts[np.isfinite(pts).all(axis=1)]

        out_header = msg.header
        out_header.frame_id = self.target_frame
        if pts.shape[0] == 0:
            self.pub.publish(pc2.create_cloud_xyz32(out_header, []))
            return

        # Apply the TF rotation + translation manually to avoid a humble bug
        # in tf2_sensor_msgs.do_transform_cloud when the source has extra fields.
        t = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_matrix(q.x, q.y, q.z, q.w)
        pts = pts @ R.T + np.array([t.x, t.y, t.z], dtype=np.float32)

        mnx, mxx, mny, mxy, mnz, mxz = self.bbox
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        inside = ((x >= mnx) & (x <= mxx) &
                  (y >= mny) & (y <= mxy) &
                  (z >= mnz) & (z <= mxz))
        kept = pts[~inside]

        self.pub.publish(pc2.create_cloud_xyz32(out_header, kept.tolist()))


def _quat_to_matrix(x, y, z, w):
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
        [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
        [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
    ], dtype=np.float32)


def main():
    rclpy.init()
    node = CropSelfHits()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
