cmake_minimum_required(VERSION 3.10)
# Project name and language
project(VREP_Interface_Example LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add include directories for DQ Robotics
include_directories(/usr/src/dqrobotics
                    /usr/include/eigen3/Eigen)

# Add the executable
add_executable(main main.cpp) 

# Link the required libraries
target_link_libraries(main dqrobotics dqrobotics-interface-vrep)