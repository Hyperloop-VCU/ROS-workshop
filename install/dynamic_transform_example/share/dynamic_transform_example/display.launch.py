from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    rviz_config = PathJoinSubstitution([
        FindPackageShare('dynamic_transform_example'),
        'rviz_config.rviz'
    ])

    return LaunchDescription([
        Node(
            package='dynamic_transform_example',
            executable='dynamic_transform_example',
            name='ExampleBroadcasterNode'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config]
        )
    ])