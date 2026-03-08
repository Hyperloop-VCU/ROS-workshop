import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from time import time_ns


class ImuSubscriber(Node):
    def __init__(self):
        # Generate a unique node name based on the time
        node_name = "basic_subscriber_node_" + str(time_ns())[11:]
        super().__init__(node_name)

        # Initialize subscriber to receive Imu messages from the topic "imu/data".
        # Whenever a message is received, self.listener_callback() is called.
        # 10 is the queue size, don't worry about it.
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Received IMU data:\n'
            f'  Orientation    -> x: {msg.orientation.x:.3f}, y: {msg.orientation.y:.3f}, '
            f'z: {msg.orientation.z:.3f}, w: {msg.orientation.w:.3f}\n'
            f'  Angular Vel    -> x: {msg.angular_velocity.x:.3f}, y: {msg.angular_velocity.y:.3f}, '
            f'z: {msg.angular_velocity.z:.3f}\n'
            f'  Linear Accel   -> x: {msg.linear_acceleration.x:.3f}, y: {msg.linear_acceleration.y:.3f}, '
            f'z: {msg.linear_acceleration.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)

    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
