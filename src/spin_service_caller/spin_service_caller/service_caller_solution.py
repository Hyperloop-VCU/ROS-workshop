"""
This is the solution code for the service caller activity.
I've included the answer here so you can run it and see the intended behavior.
You need to search the docs and figure out how to write the node yourself.

Run this node: ros2 run spin_service_caller service_caller_solution
Run YOUR node: ros2 run spin_service_caller service_caller_node
Your goal is to write service_caller_node so its behavior matches service_caller_solution.
Do NOT look at the code in this file!
"""






























































import sys

from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node


class SpinServiceCaller(Node):

    def __init__(self):
        super().__init__('spin_service_caller')
        self.cli = self.create_client(Empty, 'spin_server')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available. You need to launch imprimis\'s code first. The command is:\nros2 launch imprimis_sim_hardware imprimis_hardware.launch.py')
        
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def send_request(self, a, b):
        req = Empty.Request()
        return self.cli.call_async(req)
    
    def timer_callback(self):
        self.send_request()


def main():
    rclpy.init()

    node = SpinServiceCaller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()