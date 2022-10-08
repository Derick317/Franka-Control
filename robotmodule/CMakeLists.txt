cmake_minimum_required(VERSION 3.4)

find_package(Franka REQUIRED)

project(robotpy CXX)

add_library(robotmodule SHARED robotmodule.cpp examples_common.h examples_common.cpp)

target_link_libraries(robotmodule Franka::Franka)