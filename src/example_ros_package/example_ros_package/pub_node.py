import rclpy
from rclpy.node import Node
from time import time_ns

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        # Generate a unique node name based on the current time
        self.node_name = "basic_publisher_node_" + str(time_ns())[11:]
        super().__init__(self.node_name)

        # Initialize publisher to publish String messages to the topic "/topic".
        # The 10 is the queue size, don't worry about it
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Initialize timer to repeatedly calls self.timer_callback() every 0.2 seconds
        timer_period_s = 0.2
        self.timer = self.create_timer(timer_period_s, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hi my name is {self.node_name}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()