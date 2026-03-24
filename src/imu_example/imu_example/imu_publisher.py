import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from time import time_ns


class ImuPublisher(Node):
    def __init__(self):
        # Generate a unique node name based on the time
        node_name = "imu_publisher_" + str(time_ns())[11:]
        super().__init__(node_name)
        
        # Initialize publisher to publish Imu message to the topic "imu/data"
        # 10 is the queue size, don't worry about it
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        # Initialize timer to repeatedly call self.timer_callback() every 0.1 seconds
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Internal variable used for slow rotation
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
