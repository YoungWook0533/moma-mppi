cmake_minimum_required(VERSION 3.0.2)
project(mppi_royalpanda)

set(CMAKE_CXX_STANDARD 17)

# Find required packages
find_package(Eigen3 REQUIRED 3.3)
find_package(Franka 0.7.0 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(pinocchio REQUIRED)

set(PACKAGE_DEPENDENCIES
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    std_msgs
    roscpp
    mppi_ros
    mppi_manipulation
    roslib
    roscpp
    robot_control
    controller_interface
    franka_hw
    hardware_interface
    franka_example_controllers
    kdl_parser
    manipulation_msgs
    gazebo_msgs
    dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

generate_dynamic_reconfigure_options(
  config/pid.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_controller
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS Franka orocos_kdl control_toolbox pinocchio
)

# Include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${Franka_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${control_toolbox_INCLUDE_DIRS})

### Ros controller
add_library(${PROJECT_NAME}_controller
    src/royalpanda_controller_ros.cpp)
target_link_libraries(${PROJECT_NAME}_controller
    ${catkin_LIBRARIES}
    ${Franka_LIBRARIES}
)

### SSV model for collision avoidance
add_executable(ssv_model src/nodes/ssv_model.cpp)
target_link_libraries(ssv_model ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

### Standalone controller node using ROS to communicate
add_executable(controller_node src/nodes/controller_node.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES})

### Convert velocity commands to effort
add_executable(input_to_controller_effort_custom src/nodes/input_to_controller_effort_custom.cpp)
target_link_libraries(input_to_controller_effort_custom
    ${catkin_LIBRARIES}
    ${control_toolbox_LIBRARIES}
    pinocchio::pinocchio
)

add_executable(input_to_controller_effort src/nodes/input_to_controller_effort.cpp)
target_link_libraries(input_to_controller_effort
    ${catkin_LIBRARIES}
    ${control_toolbox_LIBRARIES}
)

### Dummy odometry publisher node for testing
add_executable(base_odometry_dummy_node src/nodes/base_odometry_dummy_node.cpp)
target_link_libraries(base_odometry_dummy_node ${catkin_LIBRARIES})

### Utility
add_executable(joint_state_filter_node src/nodes/joint_state_filter_node.cpp)
target_include_directories(joint_state_filter_node PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(joint_state_filter_node ${catkin_LIBRARIES})

### Vicon emulator node for testing
add_executable(vicon_emulator_node src/nodes/vicon_emulator_node.cpp)
target_include_directories(vicon_emulator_node PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(vicon_emulator_node ${catkin_LIBRARIES})

### State observer node
add_executable(state_observer_node src/state_observer.cpp src/nodes/state_observer_node.cpp)
target_include_directories(state_observer_node PUBLIC include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(state_observer_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
