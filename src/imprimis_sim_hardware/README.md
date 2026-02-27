# imprimis_sim_hardware
This package starts up a simulation of imprimis. It simulates a fully driveable robot with a simulated lidar, GPS, and IMU.

## launch
The launch directory contains the ros2 launch file ```imprimis_hardware.launch.py```. This starts up everything necessary for the robot to drive and output data from its sensors.
  * gui: If true, starts up Rviz. Defaults to true.
  * publish_odom_tf: If true, publishes odometry directly from the diff drive controller. This must be false when running ekf nodes, or the unfiltered and filtered odometries will conflict. Defaults to true.
  * use_controller: If true, launches teleop_twist_joy and maps its output to the controller's cmd_vel. Defaults to false.
  * hardware_type: Fake or simulated. Fake hardware is just the motors without any physics, but simulated starts the full simulation with all the sensors. Defaults to simulated.

## config
The config directory contains YAML configuration files for the robot's hard speed / acceleration limits and ros_gz_bridge.

## meshes
This directory has all the 3D assets for simulation that aren't pulled from the internet at runtime via fuel.

## worlds
This has all the simulation SDF files. SDF stands for "simulation description format"; it's the file type used to describe Gazebo simulations. It references 3D assets in the meshes directory.