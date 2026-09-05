cmake_minimum_required(VERSION 3.22)
project(vamp_r2000ic VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable SIMD instructions
if (MSVC)
    add_compile_options("/MP" "/EHsc")
else()
    add_compile_options("-O3")
endif()

# Find OMPL
find_package(ompl CONFIG QUIET)
if (NOT ompl_FOUND)
    find_package(ompl REQUIRED)
endif()

# Find Eigen3
find_package(Eigen3 CONFIG QUIET)

# Find toppra
find_package(toppra CONFIG REQUIRED)

# vamp_r2000ic Shared Library (DLL)
add_library(vamp_r2000ic SHARED
    src/vamp_r2000ic_planner.cpp
    src/vamp_r2000ic_c_api.cpp
    src/vamp_toppra.cpp
)

target_compile_definitions(vamp_r2000ic PRIVATE VAMP_R2000IC_EXPORTS)

set_target_properties(vamp_r2000ic PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/workspace/Release"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/workspace/Debug"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/workspace/Release"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/workspace/Debug"
)

target_include_directories(vamp_r2000ic PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(vamp_r2000ic PUBLIC
    ${OMPL_LIBRARIES}
    ompl::ompl
    toppra::toppra
)

# Benchmark / Test executable
add_executable(test_vamp_benchmark
    examples/test_vamp_benchmark.cpp
)

target_link_libraries(test_vamp_benchmark PRIVATE
    vamp_r2000ic
)

target_include_directories(test_vamp_benchmark PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
