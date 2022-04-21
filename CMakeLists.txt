cmake_minimum_required(VERSION 3.0.2)
project(benchmark_pipelines)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
)
include_directories(
   ${catkin_INCLUDE_DIRS}
)
