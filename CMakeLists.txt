cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(JoltC VERSION 0.1
    DESCRIPTION "C Wrapper for Jolt Physics"
    LANGUAGES CXX)

add_library(joltc STATIC
    JoltC/Enums_Test.cpp
    JoltC/JoltPhysicsC.cpp
    JoltC/JoltPhysicsC.h
    # JoltC/JoltPhysicsC_Extensions.cpp
    JoltC/JoltPhysicsC_Tests.c
)

target_compile_features(joltc PUBLIC cxx_std_17)
target_include_directories(joltc PUBLIC JoltPhysics)
target_link_libraries(joltc PUBLIC JoltPhysics)

add_subdirectory(JoltPhysics/Build)