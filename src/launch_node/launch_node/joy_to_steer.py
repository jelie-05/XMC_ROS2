import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

def map_value(val, in_min, in_max, out_min, out_max):
    """
    Maps a value from one range to another.
    val: Input value to map
    in_min, in_max: Input range
    out_min, out_max: Output range
    Returns: Mapped value
    """
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class JoyToSteer(Node):
    def __init__(self):
        super().__init__('joy_to_steer')
        self.get_logger().info('Joy to steer node started')

        # Initialize the serial connection
        self.device = serial.Serial(port='COM3', baudrate=115200, timeout=0.1)

        # Initialize your node here
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        left_stick_x = msg.axes[0]  # Assuming left stick x-axis is at index 0
        button_a = msg.buttons[0]  # Assuming button A is at index 0

        if button_a == 1: # Adding safety to avoid the stick is being moved unintentionally
            try:
                self.get_logger().info(f'Left stick x-axis: {left_stick_x}')
                mapped_value = map_value(left_stick_x, -1.0, 1.0, 0.0, 180.0)

                # Send the mapped value as a string with a newline character
                self.device.write(f"{mapped_value}\n".encode())

                # Optional: Print the mapped value for debugging
                print(f"Sending to serial: {mapped_value}")

                # Add a small delay to avoid flooding the serial connection
                time.sleep(0.1)

            except KeyboardInterrupt:
                self.get_logger().info('Keyboard interrupt detected. Exiting...')
                self.device.close()
                rclpy.shutdown()

            finally:
                # Close the serial connection when done
                self.device.close()
                rclpy.shutdown()
                print("Serial connection closed.")

def main(args=None):
    rclpy.init(args=args)
    joy_to_steer_node = JoyToSteer()
    rclpy.spin(joy_to_steer_node)

    # Destroy the node explicitly
    joy_to_steer_node.destroy_node()
    rclpy.shutdown()
    print("Node destroyed and rclpy shutdown.")