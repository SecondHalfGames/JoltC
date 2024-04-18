cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

project(JoltC VERSION 0.1
    DESCRIPTION "C Wrapper for Jolt Physics"
    LANGUAGES C CXX)

add_library(joltc STATIC
    JoltC/JoltC.h
    JoltC/JoltC.cpp
    JoltC/Enums.h
    JoltC/Functions.h
    JoltC/Test.cpp
)

target_compile_features(joltc PUBLIC cxx_std_17)
target_include_directories(joltc PUBLIC . JoltPhysics)
target_link_libraries(joltc PUBLIC Jolt)

if(MSVC)
  target_compile_options(joltc PRIVATE /W4)
else()
  target_compile_options(joltc PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(HelloWorld
    HelloWorld/main.cpp
)

target_include_directories(HelloWorld PUBLIC .)
target_link_libraries(HelloWorld PUBLIC joltc)

add_subdirectory(JoltPhysics/Build)
