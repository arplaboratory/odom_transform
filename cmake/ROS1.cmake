cmake_minimum_required(VERSION 3.8)
project(odom_transform)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp roslib sensor_msgs std_msgs nav_msgs tf nodelet)
add_definitions(-DROS_AVAILABLE=1)
catkin_package(CATKIN_DEPENDS roscpp roslib sensor_msgs std_msgs nav_msgs tf)

include_directories(src ${catkin_INCLUDE_DIRS})
add_definitions(-std=c++17)

list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${catkin_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nodelet 
  # DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# ##For node
# add_executable(transform_node src/transform.cpp)
# target_link_libraries(transform_node ${catkin_LIBRARIES} ${thirdparty_libraries})
# install(TARGETS transform_node
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## For Nodelet
## Declare a C++ library
add_library(odom_transform_nodelet
  src/transform_nodelet.cpp
  src/transform.cpp
)
target_link_libraries(odom_transform_nodelet ${catkin_LIBRARIES} ${thirdparty_libraries})
# if(catkin_EXPORTED_LIBRARIES)
#  add_dependencies(odom_transform_nodelet odom_transform_nodelet ${catkin_EXPORTED_LIBRARIES})
# endif()
install(TARGETS odom_transform_nodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

