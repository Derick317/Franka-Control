cmake_minimum_required(VERSION 3.4)

find_package(Franka REQUIRED)
include_directories("/usr/include/eigen3")
project(robotpy CXX)

add_library(robotmodule SHARED 
    robotmodule.cpp
    examples_common.cpp
)

target_link_libraries(robotmodule Franka::Franka)