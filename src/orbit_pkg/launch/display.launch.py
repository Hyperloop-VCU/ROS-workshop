from launch import LaunchDescription
from launch_ros.actions import Node

# This simple launch file starts one orbit node and rviz.

def generate_launch_description():

    dynamic_transform_example = Node(
        package='orbit_pkg',
        executable='orbit_node',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2'
    )
    return LaunchDescription([
        dynamic_transform_example,
        rviz
    ])