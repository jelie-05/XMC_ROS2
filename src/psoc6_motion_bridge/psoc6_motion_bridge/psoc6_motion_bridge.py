#!/usr/bin/env python3
import struct
import time
from typing import Optional

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu

SOF0 = 0xAA
SOF1 = 0x55

PKT_TYPE_CONFIG = 0x01
PKT_TYPE_IMU    = 0x02

G0 = 9.80665  # m/s^2 per 1 g
DEG2RAD = 3.141592653589793 / 180.0

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF

class ImuSerialBridge(Node):
    def __init__(self):
        super().__init__('imu_serial_bridge')

        # -------- Parameters --------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', 'imu/data')
        self.declare_parameter('log_config', True)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.log_config = self.get_parameter('log_config').get_parameter_value().bool_value

        # -------- Serial --------
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.0)
        # print("I'm here :)")
        self.get_logger().info(f'Opened {port} @ {baud} baud')

        # -------- QoS & Publisher --------
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Imu, topic, qos)

        # -------- Parser state --------
        self.buf = bytearray()

        # Defaults until CONFIG arrives (match your firmware)
        self.bit_width: int = 16
        self.accel_range_g: int = 8
        self.gyro_range_dps: int = 500
        self.odr_hz: int = 50

        self._recompute_scales()

        # Pump serial as fast as the executor allows
        self.timer = self.create_timer(0.0, self._spin_serial)
        self.last_config_log = 0.0

        self.msg_count = 0
        self.last_latency_ms = None
        self.create_timer(1.0, self._report_rate)

    def _report_rate(self):
        self.get_logger().info(f'IMU Rx: {self.msg_count} msgs/s'
                            + (f', last latency: {self.last_latency_ms:.2f} ms'
                                if self.last_latency_ms is not None else ''))
        self.msg_count = 0
        
    def _recompute_scales(self):
        half_scale = float(1 << (self.bit_width - 1))  # 32768 for 16-bit
        self.acc_counts_to_mps2 = (self.accel_range_g * G0) / half_scale
        self.gyr_counts_to_rads = (self.gyro_range_dps * DEG2RAD) / half_scale

    def _spin_serial(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1024)
            if data:
                self.buf.extend(data)
                self._process_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')

    def _process_buffer(self):
        while True:
            start = self._find_sof(self.buf)
            if start < 0:
                if len(self.buf) > 4:
                    del self.buf[:-4]
                return
            if start > 0:
                del self.buf[:start]
            if len(self.buf) < 3:
                return
            payload_len = self.buf[2]
            frame_len = 3 + payload_len + 2
            if len(self.buf) < frame_len:
                return

            frame = bytes(self.buf[:frame_len])
            del self.buf[:frame_len]

            payload = frame[3:-2]
            recv_crc = frame[-2] | (frame[-1] << 8)
            calc_crc = crc16_ccitt_false(payload)
            if recv_crc != calc_crc:
                self.get_logger().warn('CRC mismatch, dropping frame')
                continue

            pkt_type = payload[0]
            if pkt_type == PKT_TYPE_CONFIG:
                self._handle_config(payload)
            elif pkt_type == PKT_TYPE_IMU:
                self._handle_imu(payload)
            else:
                self.get_logger().warn(f'Unknown packet type: {pkt_type}')

    @staticmethod
    def _find_sof(buf: bytearray) -> int:
        for i in range(len(buf) - 1):
            if buf[i] == SOF0 and buf[i+1] == SOF1:
                return i
        return -1

    def _handle_config(self, payload: bytes):
        if len(payload) != 1 + 1 + 2 + 2 + 2:
            self.get_logger().warn(f'CONFIG payload size unexpected: {len(payload)}')
            return
        _, bit_width, accel_g, gyro_dps, odr = struct.unpack('<BBHHH', payload)
        self.bit_width = int(bit_width)
        self.accel_range_g = int(accel_g)
        self.gyro_range_dps = int(gyro_dps)
        self.odr_hz = int(odr)
        self._recompute_scales()
        now = time.time()
        if self.log_config and (now - self.last_config_log) > 2.0:
            self.last_config_log = now
            self.get_logger().info(
                f'CONFIG: bit={self.bit_width}, accel=±{self.accel_range_g} g, '
                f'gyro=±{self.gyro_range_dps} dps, odr={self.odr_hz} Hz'
            )

    def _handle_imu(self, payload: bytes):
        if len(payload) != 1 + 2 + 12:
            self.get_logger().warn(f'IMU payload size unexpected: {len(payload)}')
            return
        _, seq, ax, ay, az, gx, gy, gz = struct.unpack('<B H h h h h h h', payload)

        self.msg_count += 1
        lin_ax = ax * self.acc_counts_to_mps2
        lin_ay = ay * self.acc_counts_to_mps2
        lin_az = az * self.acc_counts_to_mps2

        ang_vx = gx * self.gyr_counts_to_rads
        ang_vy = gy * self.gyr_counts_to_rads
        ang_vz = gz * self.gyr_counts_to_rads

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation_covariance[0] = -1.0  # orientation unknown

        msg.angular_velocity.x = float(ang_vx)
        msg.angular_velocity.y = float(ang_vy)
        msg.angular_velocity.z = float(ang_vz)

        msg.linear_acceleration.x = float(lin_ax)
        msg.linear_acceleration.y = float(lin_ay)
        msg.linear_acceleration.z = float(lin_az)

        # self.get_logger().info(
        #         f'seq={seq}, ax={float(lin_ax)}, ay={float(lin_ay)}, az={float(lin_az)}, '
        #         f'gx={float(ang_vx)}, gy={float(ang_vy)}, gz={float(ang_vz)}'
        #     )
        
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImuSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
