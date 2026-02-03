This is an example ROS package for use in the HyperLabs ROS workshops.


## Build Steps
1. Make your workspace directory and enter it
```
mkdir -p colcon_ws/src
cd colcon_ws/
```
2. Acquire the source files. Clone this repository using git.
```
git clone https://github.com/Hyperloop-VCU/example-ros-package.git src/example-ros-package
```
3. Install the package's dependencies
```
rosdep install --from-paths src --ignore-src -r -y
```
4. Build the package using colcon
```
colcon build --symlink-install
```
5. Source the install source file to tell ROS about your new packages
```
source install/setup.bash
```
