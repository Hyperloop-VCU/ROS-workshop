import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        timer_period = 0.1  # 10 Hz, typical for an IMU
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0.0

    def timer_callback(self):
        msg = Imu()

        # Simulate orientation as a quaternion (slowly rotating around Z axis)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.t / 2.0)
        msg.orientation.w = math.cos(self.t / 2.0)

        # Simulate angular velocity (rad/s) with some noise
        msg.angular_velocity.x = random.uniform(-0.05, 0.05)
        msg.angular_velocity.y = random.uniform(-0.05, 0.05)
        msg.angular_velocity.z = 0.1 + random.uniform(-0.01, 0.01)

        # Simulate linear acceleration (m/s^2) - gravity on Z + noise
        msg.linear_acceleration.x = random.uniform(-0.2, 0.2)
        msg.linear_acceleration.y = random.uniform(-0.2, 0.2)
        msg.linear_acceleration.z = 9.81 + random.uniform(-0.1, 0.1)

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing IMU - orientation_z: {msg.orientation.z:.3f}, '
            f'accel_z: {msg.linear_acceleration.z:.2f}'
        )
        self.t += 0.1


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
