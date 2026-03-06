from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# This launch file starts up one or two turtlesim nodes and the turtle_teleop_key node required to control them.
# In this package's config directory, there are two separate config files, one for each turtlesim node.

def generate_launch_description():

    # Create a list of all the launch arguments.
    # These are options which can be passed to the launch file when you launch it for customization of the launch behavior.
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_both_nodes",
            default_value="true",
            description="If true, starts up both nodes. If false, only starts one node.",
        )
    )
    launch_both_nodes = LaunchConfiguration("start_both_nodes")


    # Retrieve the first turtlesim node's config file
    turtlesim_one_config = PathJoinSubstitution([
        FindPackageShare('launch_file_example_pkg'),
        'config',
        'turtlesim_one_params.yaml'
    ])

    # Define the first turtlesim node
    turtlesim_node_one = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="Turtle_1",
        parameters=[turtlesim_one_config]
    )

    # Retrieve the second turtlesim node's config file
    turtlesim_two_config = PathJoinSubstitution([
        FindPackageShare('launch_file_example_pkg'),
        'config',
        'turtlesim_two_params.yaml'
    ])

    # Define the second turtlesim node, which only runs if launch_both_nodes is true
    turtlesim_node_two = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="Turtle_2",
        parameters=[turtlesim_two_config],
        condition=IfCondition(PythonExpression(["'", launch_both_nodes, "'"]))
    )

    # Define the controller node, which controls both turtlesim nodes via the keyboard
    turtlesim_controller = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        name="Turtle_2"
    )

    # Package all nodes into a list and return it, along with all the launch arguments we want to declare.
    nodes = [
        turtlesim_node_one, 
        turtlesim_node_two, 
        turtlesim_controller
    ]
    return LaunchDescription(declared_arguments + nodes)