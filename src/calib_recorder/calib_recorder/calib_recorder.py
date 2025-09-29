# calib_recorder/calib_recorder.py
import os
import csv
import time
import json
from datetime import datetime

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import numpy as np
import cv2


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)
    return path


class CalibRecorder(Node):
    def __init__(self):
        super().__init__('calib_recorder')

        # Params
        self.rgb_topic = self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw').get_parameter_value().string_value
        self.rgb_info_topic = self.declare_parameter('rgb_info_topic', '/camera/camera/color/camera_info').get_parameter_value().string_value
        self.tof_gray_topic = self.declare_parameter('tof_gray_topic', '/royale/gray_image').get_parameter_value().string_value
        
        self.out_dir = self.declare_parameter('out_dir', '').get_parameter_value().string_value
        self.session = self.declare_parameter('session', '').get_parameter_value().string_value
        self.max_frames = self.declare_parameter('max_frames', 0).get_parameter_value().integer_value  # 0 means unlimited
        self.save_rate_hz = self.declare_parameter('save_rate_hz', 2.0).get_parameter_value().double_value
        self.sync_slop = self.declare_parameter('sync_slop', 0.03).get_parameter_value().double_value
        self.queue_size = self.declare_parameter('queue_size', 15).get_parameter_value().integer_value

        # Output structure
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = self.out_dir if self.out_dir else os.path.join(os.getcwd(), 'calib_dataset')
        sess = self.session if self.session else f'session_{ts}'
        self.base_dir = ensure_dir(os.path.join(base, sess))
        self.rgb_dir = ensure_dir(os.path.join(self.base_dir, 'rgb'))
        self.gray_dir = ensure_dir(os.path.join(self.base_dir, 'tof_gray'))
        self.meta_dir = ensure_dir(os.path.join(self.base_dir, 'meta'))

        # CSV manifest
        self.manifest_path = os.path.join(self.base_dir, 'manifest.csv')
        self.manifest = open(self.manifest_path, 'w', newline='')
        self.csv_writer = csv.writer(self.manifest)
        self.csv_writer.writerow([
            'index',
            'stamp_ns',
            'rgb_path',
            'gray_png_path',
            'gray_npy_path',
            'rgb_frame_id',
            'tof_frame_id'
        ])

        # CameraInfo storage
        self.last_rgb_info = None

        # Bridge
        self.bridge = CvBridge()

        # Subscribers and sync
        self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
        self.gray_sub = Subscriber(self, Image, self.tof_gray_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.gray_sub],
            queue_size=self.queue_size,
            slop=self.sync_slop,
            allow_headerless=False
        )
        self.sync.registerCallback(self.synced_cb)

        # CameraInfo subscriber
        self.create_subscription(CameraInfo, self.rgb_info_topic, self.rgb_info_cb, 10)

        # Save throttling
        self.last_save_time = 0.0
        self.period = 1.0 / max(self.save_rate_hz, 1e-6)
        self.frame_idx = 0

        self.get_logger().info(f"Recording to: {self.base_dir}")
        self.get_logger().info(f"RGB: {self.rgb_topic} | info: {self.rgb_info_topic}")
        self.get_logger().info(f"ToF depth: {self.tof_gray_topic}")

    def rgb_info_cb(self, msg: CameraInfo):
        self.last_rgb_info = msg
        self.write_cam_info_once('rgb_camera_info.json', msg)

    def write_cam_info_once(self, fname: str, msg: CameraInfo):
        path = os.path.join(self.meta_dir, fname)
        if os.path.exists(path):
            return
        data = {
            'width': msg.width,
            'height': msg.height,
            'k': list(msg.k),
            'd': list(msg.d),
            'r': list(msg.r),
            'p': list(msg.p),
            'distortion_model': msg.distortion_model,
            'header': {
                'frame_id': msg.header.frame_id
            }
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
            self.get_logger().info(f"Saved CameraInfo -> {path}")

    def synced_cb(self, rgb_msg: Image, gray_img_msg: Image):
        now = time.time()
        if (now - self.last_save_time) < self.period:
            return
        self.last_save_time = now

        # Convert messages
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB convert error: {e}')
            return

        try:
            gray_img_cv = self.bridge.imgmsg_to_cv2(gray_img_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth convert error: {e}')
            return

        # Timestamps and indices
        stamp_ns = gray_img_msg.header.stamp.sec * 1_000_000_000 + gray_img_msg.header.stamp.nanosec
        idx = self.frame_idx

        # Paths
        rgb_path = os.path.join(self.rgb_dir, f'rgb_{idx:06d}.png')
        gray_img_png_path = os.path.join(self.gray_dir, f'gray_{idx:06d}.png')
        gray_img_npy_path = os.path.join(self.gray_dir, f'gray_{idx:06d}.npy')

        # Save RGB
        cv2.imwrite(rgb_path, rgb)

        if gray_img_cv.dtype == np.uint16:
            gray_to_write = gray_img_cv
        elif gray_img_cv.dtype in (np.float32, np.float64):
            # assume amplitude in [0,1]
            gray_to_write = np.clip(gray_img_cv * 65535.0, 0, 65535).astype(np.uint16)
        else:
            # generic fallback: stretch per-frame
            gray_to_write = cv2.normalize(gray_img_cv, None, 0, 65535, cv2.NORM_MINMAX).astype(np.uint16)
        cv2.imwrite(gray_img_png_path, gray_to_write)


        # CSV line
        self.csv_writer.writerow([
            idx,
            stamp_ns,
            os.path.relpath(rgb_path, self.base_dir),
            os.path.relpath(gray_img_png_path, self.base_dir),
            os.path.relpath(gray_img_npy_path, self.base_dir),
            rgb_msg.header.frame_id,
            gray_img_msg.header.frame_id
        ])
        self.manifest.flush()

        # Stop if max frames reached
        self.frame_idx += 1
        if self.max_frames > 0 and self.frame_idx >= self.max_frames:
            self.get_logger().info('Reached max_frames, shutting down.')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CalibRecorder()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.manifest.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
