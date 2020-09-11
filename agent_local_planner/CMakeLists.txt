cmake_minimum_required(VERSION 2.8.3)
project(agent_local_planner)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(PCL REQUIRED COMPONENTS common)

include_directories(include ${PCL_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/agent_local_planner.cpp
  src/polynomial_interpolator.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${CHOLMOD_LIBRARY}
  nlopt
  ${PCL_LIBRARIES}
)

############
# BINARIES #
############
cs_add_executable(${PROJECT_NAME}_node
  src/agent_local_planner_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

