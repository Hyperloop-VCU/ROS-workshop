from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_package',
            executable='pub_node',
            name='PublisherNode'
        ),
        # Add the subscriber node yourself
    ])