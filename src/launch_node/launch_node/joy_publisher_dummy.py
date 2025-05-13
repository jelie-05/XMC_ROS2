import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time

class JoyPublisherDummy(Node):
    def __init__(self):
        super().__init__('joy_publisher_dummy')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.5, self.publish_joy_message)
        self.get_logger().info("Joy Dummy Publisher Node Initialized")

    def publish_joy_message(self):
        msg=Joy()
        msg.axes= [0.0, 1.0, -1.0]
        msg.buttons=[1,1,0]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Joy Message: Axes: {msg.axes}, Buttons: {msg.buttons}")

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisherDummy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()