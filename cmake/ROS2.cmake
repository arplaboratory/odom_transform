cmake_minimum_required(VERSION 3.5)
project(odom_transform)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_library(odom_transform_node SHARED src/transform_nodelet_ros2.cpp)
# Set the libraries' target properties
set_target_properties(odom_transform_node PROPERTIES
    COMPILE_DEFINITIONS "COMPOSITION_BUILDING_DLL"
)
# Link libraries with dependencies
ament_target_dependencies(odom_transform_node
    rclcpp
    std_msgs
    nav_msgs
    sensor_msgs
    yaml-cpp
)
target_link_libraries(odom_transform_node ${YAML_CPP_LIBRARIES})
# Register the libraries as composable nodes
rclcpp_components_register_nodes(odom_transform_node
    "transform_nodelet_ns::OvtransformNodeletClass"
)

install(TARGETS
    odom_transform_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})

ament_package()