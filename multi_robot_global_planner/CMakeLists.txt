cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_global_planner)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin_simple REQUIRED)
find_package(ompl REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)
include_directories(include)

#############
# LIBRARIES #
#############
SET(INTERPOLATORS
    src/interpolators/ramp_interpolator.cpp
    src/interpolators/polynomial_interpolator.cpp
)

cs_add_library(${PROJECT_NAME}
    src/voxblox_ompl_rrt.cpp
    src/multi_robot_global_planner.cpp
    ${INTERPOLATORS}
)
target_link_libraries(${PROJECT_NAME}
    ${OMPL_LIBRARIES}
)

############
# BINARIES #
############
cs_add_executable(${PROJECT_NAME}_node
  src/multi_robot_global_planner_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export(LIBRARIES ${OMPL_LIBRARIES})

