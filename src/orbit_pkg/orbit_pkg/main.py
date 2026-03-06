import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos, pi
from time import time, time_ns

class OrbitBroadcaster(Node):
    def __init__(self):
        node_name = "orbit_node_" + str(time_ns())[11:] # get a node ID based on the current time
        super().__init__(node_name)
        
        # declare params
        self.declare_parameter("radius", 1.0)  # 1.0 is the default value
        self.declare_parameter("revs_per_sec", 0.5)
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("orbiting_frame", "robot")
        # TODO: make the height of the orbit a parameter
        
        # print startup msg
        fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        orbiting_frame = self.get_parameter('orbiting_frame').get_parameter_value().string_value
        self.get_logger().info(f"Starting an orbit node with \'{orbiting_frame}\' orbiting \'{fixed_frame}\'. You'll need to open rviz to see it.")

        # create transform publisher and start the loop
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.broadcast_transform)  # self.broadcast_transform() runs repeatedly every 0.01s
    
    def broadcast_transform(self):

        # get current param values
        fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        orbiting_frame = self.get_parameter('orbiting_frame').get_parameter_value().string_value
        radius = self.get_parameter('radius').get_parameter_value().double_value
        revs_per_sec = self.get_parameter('revs_per_sec').get_parameter_value().double_value
        
        # Initialize message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = fixed_frame
        t.child_frame_id = orbiting_frame

        # Publish a transform from the fixed frame to the rotating frame to make it rotate around a circle
        theta = 2*pi*time()*revs_per_sec
        t.transform.translation.x = radius * cos(theta)
        t.transform.translation.y = radius * sin(theta)
        t.transform.translation.z = 0.4  # TODO: make this a parameter
        t.transform.rotation.z = sin(theta/2)
        t.transform.rotation.w = cos(theta/2)
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OrbitBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
