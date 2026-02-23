from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    turtlesim_config = PathJoinSubstitution([
        FindPackageShare('launch_file_example_pkg'),
        'config',
        'turtlesim_params.yaml'
    ])

    # Start two turtlesim nodes and pass the param file into both.
    # Declare a boolean launch argument called 'start_both' which defaults to true. If true, start up both turtlesims. If false, start only one turtlesim.
    # Refer to other launch files in this workspace and imprimis's launch files for examples.

    return LaunchDescription([
    ])