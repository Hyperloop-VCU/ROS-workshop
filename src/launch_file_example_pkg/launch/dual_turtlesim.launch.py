from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_both_nodes",
            default_value="true",
            description="Should the launch file launch both turtle sim nodes",
        )
    )
    launch_both_nodes = LaunchConfiguration("start_both_nodes")

    turtlesim_one_config = PathJoinSubstitution([
        FindPackageShare('launch_file_example_pkg'),
        'config',
        'turtlesim_one_params.yaml'
    ])
    turtlesim_two_config = PathJoinSubstitution([
        FindPackageShare('launch_file_example_pkg'),
        'config',
        'turtlesim_two_params.yaml'
    ])

 
    turtlesim_node_one = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="Turtle_1",
        parameters=[turtlesim_one_config]
    )
    turtlesim_node_two = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="Turtle_2",
        parameters=[turtlesim_two_config],
        condition=IfCondition(PythonExpression(["'", launch_both_nodes, "'"]))
    )
    turtlesim_controller = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        name="Turtle_2"
    )

    launch_arguments = [turtlesim_node_one, turtlesim_node_two, turtlesim_controller]

    return LaunchDescription(declared_arguments + launch_arguments)

###
# In launch_file_example_pkg, 
# finish the launch file which starts two turtlesim nodes 
# at the same time

# The launch file should declare a boolean launch argument 
# on whether or not to start both turtlesim nodes or just one, 
# and also passes in a YAML params file to both nodes

# I already created the package skeleton, you need to finish it
# You can refer to the dynamic_transform_example for the 
# basic launch file format / syntax, and refer to the launch files 
# in clearpath_gz/launch/ for examples on how to declare 
# launch arguments.
