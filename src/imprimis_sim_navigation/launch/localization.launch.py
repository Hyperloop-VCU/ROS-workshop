from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Declare and initialize arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_controller",
            default_value="false",
            description="Whether or not to start up the logitech controller input node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "disable_local_ekf",
            default_value="false",
            description="If false, a local EKF node fuses wheel odom with other local odom sources. If true, wheel odom is the sole local odom source.",
        )
    )
    use_controller = LaunchConfiguration("use_controller")
    disable_local_ekf = LaunchConfiguration("disable_local_ekf")

    # Get robot localization nodes config file
    roboloco_config = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_sim_navigation"),
            "config",
            "localization_nodes_config.yaml",
        ]
    )

    # Fake Lidar SLAM, just assume map = odom
    # I don't want to use real SLAM for the workshop because of the intense compute/build requirements
    fake_SLAM = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0','0','0','0','0','0','map', 'odom', "--ros-args", "--log-level", "error"],
        parameters=[{"use_sim_time": True}]
    )

    # hardware (real or simulated)
    imprimis_hardware_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('imprimis_sim_hardware'),
                'launch',
                'imprimis_hardware.launch.py'
            ])
        ]),
        launch_arguments={
                'use_controller': use_controller,
                'publish_odom_tf': disable_local_ekf
                }.items(),
    )
    
    # Local EKF node for local odom fusion from multiple sources
    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        parameters=[roboloco_config, {"use_sim_time": True}],
        remappings=[('/odometry/filtered', '/odometry/filtered/local')],
        arguments=["--ros-args", "--log-level", "warn"],
        condition=UnlessCondition(disable_local_ekf)
    )

    # Helper node to wait for odom -> base_link, exits once it's available
    wait_for_odom_tf_and_lidar = Node(
        package="imprimis_sim_utils",
        executable="wait_for_tf",
        parameters=[{
            "source_frame": "odom",
            "target_frame": "base_link",
        }]
    )

    # Fake SLAM, after hardware/EKF has made odom -> base_link available
    fake_SLAM_start = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_odom_tf_and_lidar,
            on_exit=[
                fake_SLAM
            ]
        )
    )
   

    return LaunchDescription(declared_arguments + [
        imprimis_hardware_launch_include,
        local_ekf_node,
        wait_for_odom_tf_and_lidar,
        fake_SLAM_start
    ])