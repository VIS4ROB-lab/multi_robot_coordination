cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_simulation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(PCL REQUIRED COMPONENTS common filters)

add_definitions(-std=c++11)
include_directories(include ${PCL_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
# Odometry transformator
cs_add_library(odometry_transform_publisher
    src/odometry_transform_publisher.cpp
)
target_link_libraries(odometry_transform_publisher
    ${CATKIN_LIBRARIES}
)

# Pointcloud transformator
cs_add_library(pointcloud_transformator
    src/pointcloud_transformator.cpp
)
target_link_libraries(pointcloud_transformator
    ${CATKIN_LIBRARIES}
    ${PCL_LIBRARIES}
)

# VINS initializer
cs_add_library(vins_initializer
    src/vins_initializer.cpp
)
target_link_libraries(vins_initializer
    ${CATKIN_LIBRARIES}
)

# GPS initializer
cs_add_library(gps_initializer
    src/gps_pose_graph_initializer.cpp
)
target_link_libraries(gps_initializer
    ${CATKIN_LIBRARIES}
)

# Command Interceptor
cs_add_library(command_interceptor
    src/command_interceptor.cpp
)
target_link_libraries(command_interceptor
    ${CATKIN_LIBRARIES}
)

############
# BINARIES #
############
# Odometry transformator
cs_add_executable(odometry_transform_publisher_node
    src/odometry_transform_publisher_node.cpp
)
target_link_libraries(odometry_transform_publisher_node
    odometry_transform_publisher
)

# Pointcloud transformator
cs_add_executable(pointcloud_transformator_node
    src/pointcloud_transformator_node.cpp
)
target_link_libraries(pointcloud_transformator_node
    pointcloud_transformator
)

# VINS initializer
cs_add_executable(vins_initializer_node
    src/vins_initializer_node.cpp
)
target_link_libraries(vins_initializer_node
    vins_initializer
)

# GPS initializer
cs_add_executable(gps_initializer_node
    src/gps_pose_graph_initializer_node.cpp
)
target_link_libraries(gps_initializer_node
    gps_initializer
)

# GPS interceptor
cs_add_executable(gps_interceptor_node
    src/gps_interceptor_node.cpp
)
target_link_libraries(gps_interceptor_node
    ${CATKIN_LIBRARIES}
)

# Command Interceptor
cs_add_executable(command_interceptor_node
    src/command_interceptor_node.cpp
)
target_link_libraries(command_interceptor_node
    command_interceptor
)

##########
# EXPORT #
##########
cs_install()
cs_export()

