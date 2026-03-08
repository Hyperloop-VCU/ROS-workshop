from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_example',
            executable='imu_publisher',
            name='ImuPublisher'
        ),
        Node(
            package='imu_example',
            executable='imu_subscriber',
            name='ImuSubscriber'
        ),
    ])
