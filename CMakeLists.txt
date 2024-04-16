cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(JoltC VERSION 0.1
    DESCRIPTION "C Wrapper for Jolt Physics"
    LANGUAGES CXX)

add_library(joltc STATIC
    JoltC/JoltC.h
    JoltC/JoltC.cpp
    JoltC/Enums.h
    JoltC/Functions.h
    JoltC/Test.cpp
)

target_compile_features(joltc PUBLIC cxx_std_17)
target_include_directories(joltc PUBLIC . JoltPhysics)
target_link_libraries(joltc PUBLIC JoltPhysics)

add_subdirectory(JoltPhysics/Build)