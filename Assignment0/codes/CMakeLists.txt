cmake_minimum_required (VERSION 3.0)
project (Transformation)
 
find_package (Eigen3 3.3 REQUIRED)
 
add_executable (Transformation main.cpp)
target_link_libraries (Transformation Eigen3::Eigen)
