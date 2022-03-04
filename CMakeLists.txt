cmake_minimum_required(VERSION 3.0.2)
project(benchmark_pipelines)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  eigen_conversions
  eigen_matrix_utils
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  name_sorting
  roscpp
  moveit_planning_helper
  rosparam_utilities
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs eigen_conversions eigen_matrix_utils moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools name_sorting roscpp moveit_planning_helper rosparam_utilities
  DEPENDS
)
include_directories(
   ${catkin_INCLUDE_DIRS}
)

add_executable(        generate_queries src/generate_queries.cpp)
add_dependencies(      generate_queries ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( generate_queries
 ${catkin_LIBRARIES}
)

add_executable(        run_queries src/run_queries.cpp)
add_dependencies(      run_queries ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( run_queries
 ${catkin_LIBRARIES}
)
