cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME wheel_leg)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# User application, BSP and base library sources
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/*.s
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.s
    ${CMAKE_CURRENT_SOURCE_DIR}/BaseLib/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BaseLib/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BaseLib/*.s
)

set(USER_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/APP
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/BaseLib
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/TinyMPC/include/Eigen
)

file(GLOB_RECURSE USER_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BaseLib/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/BaseLib/*.hpp
)

foreach(USER_HEADER ${USER_HEADERS})
    get_filename_component(USER_HEADER_DIR ${USER_HEADER} DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS ${USER_HEADER_DIR})
endforeach()

list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_LOOPUNROLL
    ARM_MATH_MATRIX_CHECK
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    arm_cortexM7lfsp_math

    # Add user defined libraries
)
