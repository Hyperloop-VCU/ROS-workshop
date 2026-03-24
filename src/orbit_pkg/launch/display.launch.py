from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# This simple launch file starts one orbit node and rviz.

def generate_launch_description():

    # Get rviz configuration file 
    rviz_config_file = PathJoinSubstitution([FindPackageShare('orbit_pkg'), "rviz_config.rviz"])

    # Start orbit node
    orbit_node = Node(
        package='orbit_pkg',
        executable='orbit_node',
    )

    # Start rviz and pass in the config file
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "warn"],
    )
    return LaunchDescription([
        orbit_node,
        rviz
    ])