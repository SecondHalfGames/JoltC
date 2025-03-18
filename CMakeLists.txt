cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

# Enable assertions in JoltPhysics
option(USE_ASSERTS "Enable asserts" OFF)

# When turning this option on, the library will be compiled using doubles for positions. This allows for much bigger worlds.
option(DOUBLE_PRECISION "Use double precision math" OFF)

# Number of bits to use in ObjectLayer. Can be 16 or 32.
option(OBJECT_LAYER_BITS "Number of bits in ObjectLayer" 16)

include(CMakeDependentOption)

# Ability to toggle between the static and DLL versions of the MSVC runtime library
# Windows Store only supports the DLL version
cmake_dependent_option(USE_STATIC_MSVC_RUNTIME_LIBRARY "Use the static MSVC runtime library" ON "MSVC;NOT WINDOWS_STORE" OFF)

if (USE_STATIC_MSVC_RUNTIME_LIBRARY)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Only do this when we're at the top level, see: https://gitlab.kitware.com/cmake/cmake/-/issues/24181
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
endif()

project(JoltC VERSION 0.1
    DESCRIPTION "C Wrapper for Jolt Physics"
    LANGUAGES C CXX)

add_library(joltc STATIC
    # C Interface
    JoltC/JoltC.h
    JoltC/Enums.h
    JoltC/Functions.h

    # C++ Implementation
    JoltCImpl/Test.cpp
    JoltCImpl/JoltC.cpp
)

target_compile_features(joltc PUBLIC cxx_std_17)
target_include_directories(joltc PUBLIC . JoltPhysics)
target_link_libraries(joltc PUBLIC Jolt)

install(TARGETS joltc DESTINATION lib)

if(MSVC)
  target_compile_options(joltc PRIVATE /W4)

  # Enable debug symbols
  target_compile_options(joltc PRIVATE /Zi)

  # Ignore 'unreferenced function with internal linkage'
  target_compile_options(joltc PRIVATE /wd4505)
else()
  target_compile_options(joltc PRIVATE -Wall -Wextra -Wpedantic -fno-rtti)
endif()

if (DOUBLE_PRECISION)
    target_compile_definitions(joltc PUBLIC JPC_DOUBLE_PRECISION)
endif()

if (OBJECT_LAYER_BITS)
    target_compile_definitions(joltc PUBLIC JPC_OBJECT_LAYER_BITS=${OBJECT_LAYER_BITS})
endif()

if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    add_executable(HelloWorld
        HelloWorld/main.cpp
    )

    # cxx_std_20 gives us designated initializers, bringing us closer to Actual C.
    target_compile_features(HelloWorld PRIVATE cxx_std_20)

    # Eventually we should switch back to Actual C:
    # set_property(TARGET HelloWorld PROPERTY C_STANDARD 23)

    target_include_directories(HelloWorld PUBLIC .)
    target_link_libraries(HelloWorld PUBLIC joltc)
endif()

add_subdirectory(JoltPhysics/Build)