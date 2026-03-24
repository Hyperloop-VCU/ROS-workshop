#!/bin/bash

# Ensure this bash script's WD matches its location
THIS_SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
cd $THIS_SCRIPT_DIR

# Get ros distro by checking /opt/ros/*. Assumes only one distro is installed
ROS_DISTRO_FOUND="$(ls /opt/ros 2>/dev/null | head -n1)"

# If the variable is empty, ROS is not installed
if [ -z "$ROS_DISTRO_FOUND"  ]; then
	
	echo "ROS is not installed. Attempting ROS install."

	# Validate ubuntu version
	. /etc/os-release
	MAJOR_VERSION=${VERSION_ID%.*}
	if ((MAJOR_VERSION == 22)); then
		echo "Detected Ubuntu 22.04, will install ros2 humble."
	elif ((MAJOR_VERSION == 24)); then
		echo "Detected Ubuntu 24.04, will install ros2 jazzy."	

	else
		echo "Error: Ubuntu version unsupported. Printing current Ubuntu version and exiting."
		lsb_release -a
		exit 1
	fi
	
	# Prepare for ROS install
	sudo apt install software-properties-common -y
	sudo add-apt-repository universe
	sudo apt update && sudo apt install curl -y
	export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
	curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
	sudo dpkg -i /tmp/ros2-apt-source.deb
	sudo apt update && sudo apt install ros-dev-tools -y
	sudo apt update && sudo apt upgrade
	
	# Install correct ROS version
	MAJOR_VERSION=${VERSION_ID%.*}
	if ((MAJOR_VERSION == 22)); then
		sudo apt install ros-humble-desktop
	elif ((MAJOR_VERSION == 24)); then
		sudo apt install ros-jazzy-desktop
	fi
	echo "ROS installation completed."

# If the variable is not empty, ROS is installed	
else
	echo "No need to install ROS, ROS $ROS_DISTRO_FOUND is already installed"
fi


# Remove previous alias definition in bashrc (check for ROS-workshop)
if grep -q "ROS-workshop" ~/.bashrc; then
    echo "Removing previous ROS-workshop alias definition from bashrc."
    sed -i '/ROS-workshop/,/^[[:space:]]*fi[[:space:]]*$/d' ~/.bashrc
fi

# Add new path to alias definitions to bashrc
echo "Adding ROS-workshop alias definition to bashrc."
echo "if [ -f $THIS_SCRIPT_DIR/.workshop_aliases ]; then
   . $THIS_SCRIPT_DIR/.workshop_aliases
fi" >> ~/.bashrc


# Do rosdep
echo "Running rosdep."
sudo rosdep init 
rosdep update --rosdistro humble
rosdep update --rosdistro jazzy
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# Initial build
echo "Building workspace."
source /opt/ros/$ROS_DISTRO_FOUND/setup.bash
colcon build && source install/setup.bash

# Done
echo "Workshop setup completed! :)"
