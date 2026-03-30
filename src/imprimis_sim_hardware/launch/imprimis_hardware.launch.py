# This launch file starts up everything necessary to control a virtual imprimis.
# Optionally, it can starts up a service server node that hosts a simple service to spin the robot in place.

# IMPORTS
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Every python launch file has this function, called generate_launch_description().
# This is like the main() of the launch file, all the code is within this function.
# The code can be hard to read at first, but most launch files use the same structure. It generally gets easier to understand the more you look at it.
def generate_launch_description():

    # Declare arguments
    # All the launch arguments are declared here, at the top of the launch file.
    # To add another argument, simply copy/paste one of the "declared_arguments.append" blocks and replace its values with what you want.
    # You'll also need to add an "argument = LaunchConfiguration('argument')" block at the bottom of this section.
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_spin_server",
            default_value="true",
            choices=("true", "false"),
            description="If true, starts up the spin server node. If false, does not start it."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "another_random_argument",
            default_value="true",
            choices=("true", "false"),
            description="This is a placeholder argument used to show how declaring arguments work"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "second_random_argument",
            default_value="fake",
            choices=("real", "fake", "simulated"),
            description="This is yet another placeholder argument used to show how declaring arguments work"
        )
    )
    start_spin_server = LaunchConfiguration("start_spin_server")
    another_random_argument = LaunchConfiguration("another_random_argument")  # these are greyed out because they aren't used
    second_random_argument = LaunchConfiguration("second_random_argument")


    # After declaring the arguments, code lists out all the nodes that are started in this launch file.
    # Launch files can do more than just launch nodes, as you'll soon see. They can even launch other launch files!

    # In addition to starting nodes, launch files can also run commands in the terminal directly.
    # This command will parse the robot's URDF and output it as a string.
    # The command's output is stored in the 'robot_description' variable.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("imprimis_sim_description"), "urdf", "diffbot.urdf.xacro"]
            )
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    
    # This is the main ROS2 control node.
    # ROS2 control consists of controllers and hardware interfaces. 
    # Hardware interfaces are exactly what they sound like - interfaces that can be used to interact with hardware.
    # Hardware interfaces can either be writing (commanding motors, turning on LEDs, etc) or reading (getting motor velocities, battery voltage, etc).
    # You don't interact with hardware interfaces yourself - that job goes to special nodes called controllers.
    # The controllers expose hardware interfaces as topics, which other nodes can use to interact with the robot.
    # Controllers are spawned in dynamically by this "master node", called the controller manager.
    controller_manager_config = PathJoinSubstitution([FindPackageShare("imprimis_sim_hardware"), "config", "diffbot_controllers.yaml"])
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_manager_config],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )


    # This is the controller which accepts velocity commands on the topic '/diffbot_base_controller/cmd_vel'.
    # It uses this topic to command the robot's motors by writing data to the hardware interface.
    # We don't start the controller ourselves, we launch this "spawner" node which asks the controller manager to spawn it in for us.
    motor_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": False}],
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],
    )

    # This is the controller that publishes the states of the wheels to the topic /joint_states.
    # It does this by reading data from the hardware interface.
    # We don't start the controller ourselves, we launch this "spawner" node which asks the controller manager to spawn it in for us.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": False}],
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],  
    )

    # Combines the joint state data from joint_state_broadcaster with the URDF
    # Outputs all the transforms of the robot (base_link -> wheels, base_link -> chassis, base_link -> sensors, etc).
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": False}],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    # Rviz; for visualizing data.
    rviz_config_file = PathJoinSubstitution([FindPackageShare("imprimis_sim_description"), "rviz", "diffbot.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "warn"],
        parameters=[{"use_sim_time": False}],
    )

    # Spin server; this is the node that creates the service to spin the robot around.
    # The spin server publishes to diffbot_base_controller/cmd_vel.
    # The motor controller subscribes to this topic and writes to the hardware interface to move the robot around.
    spin_server = Node(
        package="imprimis_sim_hardware",
        executable="spin_server",
        condition=IfCondition(start_spin_server)
    )

    # Launch everything. 
    # In launch files, all the nodes and arguments must be packaged together in a single list.
    # This list must be wrapped in a LaunchDescription and returned by this generate_launch_description() function.
    # This tells ROS what to launch and what the arguments are.
    nodes = [
        controller_manager,
        motor_controller_spawner,
        joint_state_broadcaster_spawner,
        robot_state_publisher,
        rviz,
        spin_server  # only starts if start_spin_server = true
    ]
    return LaunchDescription(declared_arguments + nodes)  # declared_arguments was defined at the top