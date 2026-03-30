cmake_minimum_required(VERSION 3.16)
project(imprimis_sim_hardware LANGUAGES CXX)

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# declare all dependencies 
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  std_srvs
)

# find all dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# Add executable and use ament to add dependencies to it
add_executable(spin_server src/spin_server.cpp)
ament_target_dependencies(spin_server rclcpp std_srvs geometry_msgs)

# INSTALL
install(
  DIRECTORY launch config
  DESTINATION share/imprimis_sim_hardware
)

install(
  TARGETS spin_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

## EXPORTS
ament_package()
