from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Get URDF via xacro and pass arguments to it
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

    
    # controller manager
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_sim_hardware"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": False}],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("imprimis_sim_description"), "rviz", "diffbot.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "warn"],
        parameters=[{"use_sim_time": False}],
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": False}],
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],  
    )

    motor_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": False}],
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],
    )


    things_to_launch = [
        controller_manager,
        motor_controller_spawner,
        joint_state_broadcaster_spawner,
        robot_state_publisher,
        rviz,
    ]

    return LaunchDescription(things_to_launch)