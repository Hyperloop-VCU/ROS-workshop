import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos, pi
import random
from time import time

class ExampleTfBroadcaster(Node):
    def __init__(self):
        super().__init__('example_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.broadcast_transform)
        self.radius = 2.0
        self.revs_per_sec = 0.1

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # Publish a transform from world to base_link to make it rotate around a circle
        theta = 2*pi*time()*self.revs_per_sec
        t.transform.translation.x = self.radius * cos(theta)
        t.transform.translation.y = self.radius * sin(theta)
        t.transform.translation.z = 0.0
        t.transform.rotation.z = sin(theta/2)
        t.transform.rotation.w = cos(theta/2)
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = ExampleTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
