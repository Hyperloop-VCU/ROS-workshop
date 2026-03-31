"""
This node sets up a timer to repeatedly call the function "call_spin_service".
Your task is to implement this function, and make any other necessary changes to the code so it works.
The service you want to call is "spin_service", and it is of type "Empty".

Read the docs and figure this one out for yourself!
https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
"""




from std_srvs.srv import Empty  # Import the service type, it's of type "empty"
import rclpy
from rclpy.node import Node


class SpinServiceCaller(Node):

    def __init__(self):
        super().__init__('spin_service_caller')
        self.get_logger().info("Starting node to call the service 'spin_service' repeatedly.")
        # You need to add something here
        
        timer_period = 0.5
        self.numCalls = 0
        self.timer = self.create_timer(timer_period, self.call_spin_service)

    def call_spin_service(self):
        self.numCalls += 1
        self.get_logger().info(f"({self.numCalls}) Calling the spin service. The robot should be moving.")
        # You need to add something here


def main():
    rclpy.init()

    node = SpinServiceCaller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()