cmake_minimum_required(VERSION 3.16)
project(Core VERSION 0.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Global build settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generate compile_commands.json so clangd/clang-tidy pick up include paths.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# Eigen3 dependency
#   - Prefer the system Eigen3 (find_package -> Eigen3::Eigen).
#   - Fall back to the Eigen copy bundled with TinyMPC if not installed.
# ---------------------------------------------------------------------------
find_package(Eigen3 QUIET NO_MODULE)

if(TARGET Eigen3::Eigen)
    message(STATUS "Core: using system Eigen3 (found by find_package)")
else()
    set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/TinyMPC/include/Eigen")
    if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
        message(FATAL_ERROR
            "Eigen3 was not found. Install libeigen3-dev or provide the bundled "
            "Eigen under third_party/TinyMPC/include/Eigen.")
    endif()
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
    message(STATUS "Core: using bundled Eigen at ${EIGEN3_INCLUDE_DIR}")
endif()

# ---------------------------------------------------------------------------
# autodiff dependency
#   autodiff is header-only, but its interface target provides the include
#   path and C++17 requirement used by LegCalc.
# ---------------------------------------------------------------------------
if(NOT TARGET autodiff::autodiff)
    set(AUTODIFF_BUILD_PYTHON OFF CACHE BOOL "Disable autodiff Python bindings" FORCE)
    set(AUTODIFF_BUILD_EXAMPLES OFF CACHE BOOL "Disable autodiff examples" FORCE)
    set(AUTODIFF_BUILD_DOCS OFF CACHE BOOL "Disable autodiff docs" FORCE)
    add_subdirectory(
        "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/autodiff"
        "${CMAKE_CURRENT_BINARY_DIR}/autodiff"
        EXCLUDE_FROM_ALL
    )
endif()

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(core
    src/controllerbase.cpp
    src/controller.cpp
    src/controller_at.cpp
    src/leg_calc.cpp
    src/pid.cpp
)
add_library(Core::core ALIAS core)

target_include_directories(core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(core PUBLIC Eigen3::Eigen autodiff::autodiff)

# Allow the static library to be linked into shared libraries later.
set_target_properties(core PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(core PRIVATE -Wall -Wextra)
endif()

# The STM32 Debug configuration otherwise emits the Eigen Hamiltonian solver
# at -O0, which exceeds the device flash size. Keep symbols available while
# using size optimization for this platform-independent library.
if(CMAKE_SYSTEM_NAME STREQUAL "Generic")
    target_compile_options(core PRIVATE -Os)
endif()

# ---------------------------------------------------------------------------
# Test application
# ---------------------------------------------------------------------------
option(CORE_BUILD_TESTS "Build the Core test application" ON)
if(CORE_BUILD_TESTS)
    enable_testing()

    add_executable(core_test src/test.cpp)
    target_link_libraries(core_test PRIVATE core)

    add_test(NAME core_test COMMAND core_test)
endif()

# ---------------------------------------------------------------------------
# Install & export (so downstream projects can use find_package(Core))
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS core
    EXPORT CoreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY inc/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT CoreTargets
    FILE CoreTargets.cmake
    NAMESPACE Core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Core
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CoreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/CoreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Core
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/CoreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/CoreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/CoreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Core
)
