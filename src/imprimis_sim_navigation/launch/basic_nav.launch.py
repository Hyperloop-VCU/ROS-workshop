from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():

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
            "world",
            default_value="warehouse",
            description="World used for simulation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("imprimis_sim_navigation"), "config", "nav2_blank_map", "nav2_blank_params.yaml"]
            ),
            description="Full path to the Nav2 parameters YAML.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "map_yaml",
            default_value=PathJoinSubstitution(
                [FindPackageShare("imprimis_sim_navigation"), "config", "nav2_blank_map", "bigger_blank.yaml"]
            ),
            description="Full path to the map YAML for map_server.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "disable_local_ekf",
            default_value='false',
            description="If false, a local EKF node fuses wheel odom with other local odom sources. If true, wheel odom is the sole local odom source.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "autostart_nav2",
            default_value="true",
            description="Autostart Nav2 lifecycle nodes.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dummy",
            default_value="true",
            choices=("true",),
            description="For some reason, you can\'t pass a bool directly to a launch argument in a launch file. So I did this.",
        )
    )

    use_controller = LaunchConfiguration("use_controller")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    map_yaml = LaunchConfiguration("map_yaml")
    autostart_nav2 = LaunchConfiguration("autostart_nav2")
    disable_local_EKF = LaunchConfiguration("disable_local_ekf")
    true = LaunchConfiguration("dummy")

    # hardware and localization (simulated only)
    localization_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("imprimis_sim_navigation"),
                "launch",
                "localization.launch.py"
            ])
        ]),
        launch_arguments={
            "use_controller": use_controller,
            "disable_local_EKF": disable_local_EKF
        }.items(),
    )

    # Helper node to wait for map -> odom tf and exit once it's available
    wait_for_map_odom_tf = Node(
        package="imprimis_sim_utils",
        executable="wait_for_tf",
        parameters=[{
            "source_frame": "map",
            "target_frame": "odom"
        }]
    )

    # map server
    map_server_node = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_odom_tf,
            on_exit = [
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    output="screen",
                    parameters=[{
                        "yaml_filename": map_yaml, 
                        "use_sim_time": True
                    }],
                )
            ]
        )
    )

    # lifecycle manager for map server
    lifecycle_manager_map = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_odom_tf,
            on_exit = [
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_map",
                    output="screen",
                    parameters=[{
                        "autostart": True,
                        "node_names": ["map_server"],
                        "use_sim_time": True
                    }],
                )
            ]
        )
    )

    # nav2 navigation stack (planner/controller/bt/costmaps)
    nav2_navigation_launch = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_odom_tf,
            on_exit = [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([FindPackageShare("imprimis_sim_navigation"), "launch", "nav2_minimal_bringup.launch.py"])
                    ),
                    launch_arguments={
                        "params_file": nav2_params_file,
                        "autostart": autostart_nav2,
                        "use_sim_time": true
                    }.items(),
                )
            ]
        )
    )

    return LaunchDescription(declared_arguments + [
        localization_launch_include,

        # wait for map -> odom tf before launching nav2 stack
        wait_for_map_odom_tf,

        # nav2 stack
        map_server_node,
        lifecycle_manager_map,
        nav2_navigation_launch,
    ])