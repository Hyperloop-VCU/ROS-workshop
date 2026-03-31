"""
This is the solution code for the service caller activity.
I've included the answer here so you can run it and see the intended behavior.
You need to search the docs and figure out how to write the node yourself.

Run this node: ros2 run spin_service_caller service_caller_solution
Run YOUR node: ros2 run spin_service_caller service_caller_node
Your goal is to write service_caller_node so its behavior matches service_caller_solution.
Do NOT look at the code in this file!
"""































































from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node


class SpinServiceCaller(Node):

    def __init__(self):
        super().__init__('spin_service_caller')
        self.get_logger().info("Starting node to call the service 'spin_service' repeatedly.")
        self.client = self.create_client(Empty, "spin_service")  # ADDITION: create the client that calls the service
        
        timer_period = 0.5
        self.numCalls = 0
        self.timer = self.create_timer(timer_period, self.call_spin_service)

    def call_spin_service(self):
        self.numCalls += 1
        self.get_logger().info(f"({self.numCalls}) Calling the spin service. The robot should be moving.")
        self.client.call_async(Empty.Request())  # ADDITION: Call the service with an empty request


def main():
    rclpy.init()

    node = SpinServiceCaller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()