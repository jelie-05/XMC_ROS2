import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

def map_steer_value(val, in_min, in_max, out_min, out_max):
    """
    Maps a value from one range to another.
    val: Input value to map
    in_min, in_max: Input range
    out_min, out_max: Output range
    Returns: Mapped value
    """
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def map_cmd_value(val_y, steering):
    
    if val_y < -0.5:    # Backward
        cmd = bytes([steering, 1, 0])
    elif val_y > 0.5:
        cmd = bytes([steering, 1, 1])
    else:
        cmd = bytes([steering, 0, 0])
    
    return cmd

class JoyToSteer(Node):
    def __init__(self):
        super().__init__('joy_to_steer')
        self.get_logger().info('Joy to steer node started')

        # Initialize the serial connection
        # self.device_port = self.declare_parameter('device_port', '/dev/ttyUSB0').value
        self.device = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)
        self.get_logger().info('Device is initialized')

        # Initialize your node here
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        # self.get_logger().info('Received joystick message')
        
        left_stick_x = msg.axes[0]  # Assuming left stick x-axis is at index 0
        right_stick_y = msg.axes[3] # Assuming right stick y-axis is at index 2
        button_a = msg.buttons[0]  # Assuming button A is at index 0

        if button_a == 1: # Adding safety to avoid the stick is being moved unintentionally
            self.get_logger().info(f'Left stick x-axis: {left_stick_x}')
            steering = map_steer_value(left_stick_x, -1.0, 1.0, 0.0, 180.0)
            steering = int(steering)

            cmd = map_cmd_value(right_stick_y, steering)

            # self.device.write(f"{steering}\n".encode())
            self.device.write(cmd)

            # Optional: Print the mapped value for debugging
            self.get_logger().info(f"Mesage sent to serial device. Value: {steering}; cmd: {cmd}")

            # Add a small delay to avoid flooding the serial connection
            time.sleep(0.1)
        else:
            cmd = map_cmd_value(0,0)

        def destroy_node(self):
            if hasattr(self, 'device') and self.device.is_open:
                self.device.close()
                self.get_logger().info('Serial device closed')
            super().destroy_node()
            

def main(args=None):
    rclpy.init(args=args)
    joy_to_steer_node = JoyToSteer()
    rclpy.spin(joy_to_steer_node)

    # Destroy the node explicitly
    joy_to_steer_node.destroy_node()
    rclpy.shutdown()
    print("Node destroyed and rclpy shutdown.")