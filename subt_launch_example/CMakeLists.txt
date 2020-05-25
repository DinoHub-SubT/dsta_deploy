cmake_minimum_required(VERSION 2.8.3)
project(subt_launch_example)

# include common cmake files
include("${PROJECT_SOURCE_DIR}/../common/common_cmake/SubtIncludes.cmake.in")
include("${PROJECT_SOURCE_DIR}/cmake/subt_launch_example-extras.cmake")

# find basic catkin find package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# create a basic catkin package
catkin_package(
# INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
# CATKIN_DEPENDS roscpp rospy std_msgs
# DEPENDS system_lib
)

# include basic catkin include directories
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

# add a dummy library -- to force a post build command, source_launch_params
add_library(${PROJECT_NAME} src/dummy.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# convert a relative path to an absolute path
get_filename_component(
  abs_catkin_devel_path "${CMAKE_SOURCE_PARAMS_PATH}"
  REALPATH BASE_DIR "${PROJECT_SOURCE_DIR}")

# run post build command -- source the launch params
source_launch_params(
  TARGET      ${PROJECT_NAME}
  SETUP_PATH  ${abs_catkin_devel_path}
  ENV         setup.bash setup.zsh
)
