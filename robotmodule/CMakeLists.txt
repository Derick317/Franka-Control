cmake_minimum_required(VERSION 3.4)

find_package(Franka REQUIRED)

project(robotpy CXX)

add_executable(test_gripper test_gripper.cpp)
target_link_libraries(test_gripper Franka::Franka ${CMAKE_THREAD_LIBS_INIT})

add_library(robotmodule SHARED 
    robotmodule.cpp
    examples_common.cpp
)

target_link_libraries(robotmodule Franka::Franka)