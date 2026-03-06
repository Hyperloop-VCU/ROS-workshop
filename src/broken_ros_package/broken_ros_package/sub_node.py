import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import time_ns


class MinimalSubscriber(Node):

    def __init__(self):
        # Generate a unique node name based on the current time
        node_name = "basic_subscriber_node_" + str(time_ns())[11:]
        super().__init__(node_name)

        # Initialize subscription to receive String messages from the topic "/tupac"
        # Whenever a message is received, self.listener_callback() is called.
        # 10 is the queue size, don't worry about it     
        self.subscription = self.create_subscription(String, 'tupac', self.listener_callback, 10)

    def listener_callback(self, msg):
        pass
        #self.get_logger().info(f'I heard: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()