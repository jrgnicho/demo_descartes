cmake_minimum_required(VERSION 2.8.3)
project(plan_and_run)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_planner
  moveit_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plan_and_run
#  CATKIN_DEPENDS descartes_core descartes_planner trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(plan_and_run
#   src/${PROJECT_NAME}/plan_and_run.cpp
# )

## Declare a cpp executable
# add_executable(plan_and_run_node src/plan_and_run_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(plan_and_run_node plan_and_run_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(plan_and_run_node
#   ${catkin_LIBRARIES}
# )


