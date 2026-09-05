cmake_minimum_required(VERSION 3.22)
set(VCPKG_ROOT "D:/build/vcpkg" CACHE PATH "vcpkg root")
set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE FILEPATH "")
# Manifest-mode layout: packages live in vcpkg_installed, not classic installed/
set(VCPKG_INSTALLED_DIR "${VCPKG_ROOT}/vcpkg_installed" CACHE PATH "")

# Extract package name and version
# find_package(ros_industrial_cmake_boilerplate REQUIRED)

project(Test VERSION 0.0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/MP>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/MP>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/EHsc>")
endif()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
# 添加以下行以限制只支持Debug和Release配置
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)

message(STATUS "############################# Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Configuration types: ${CMAKE_CONFIGURATION_TYPES}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/workspace/Release")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/workspace/Debug")


if (WIN32)
  add_definitions(-DNOMINMAX)
endif()

if (MSVC)
    add_compile_options(/openmp)
endif()

# find_package(RVSBase REQUIRED)
# find_package(ros_industrial_cmake_boilerplate REQUIRED)

# OCR Tesseract also installs share/tesseract; load robotics first so
# tesseract_DIR points at tesseract-robotics (imported targets stay tesseract::*).
find_package(tesseract-robotics CONFIG REQUIRED)
find_package(tesseract REQUIRED COMPONENTS common geometry scene_graph state_solver collision srdf urdf kinematics environment visualization support)
find_package(Boost REQUIRED COMPONENTS program_options filesystem serialization system)
find_package(tesseract_planning REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(GTest CONFIG REQUIRED)

set(TESSERACT_VCPKG_PREFIX "${VCPKG_INSTALLED_DIR}/x64-windows")
set(TESSERACT_TEST_SHARE_DIR "${TESSERACT_VCPKG_PREFIX}/share")

add_library(robot_planner STATIC
  src/robot_planner.cpp
)

add_library(${PROJECT_NAME} STATIC
  src/example.cpp
  src/basic_cartesian_example.cpp
  src/car_seat_example.cpp
  src/freespace_hybrid_example.cpp
  src/freespace_ompl_example.cpp
  src/glass_upright_example.cpp
  src/online_planning_example.cpp
  src/pick_and_place_example.cpp
  src/puzzle_piece_auxillary_axes_example.cpp
  src/puzzle_piece_example.cpp
  src/scene_graph_example.cpp
  )

target_link_libraries(
  robot_planner
  PUBLIC tesseract::environment
         tesseract::motion_planners
         tesseract::motion_planners_ompl
         tesseract::motion_planners_trajopt
         tesseract::motion_planners_trajopt_ifopt
         trajopt::trajopt_sqp
         trajopt::trajopt_ifopt
         tesseract::task_composer
         tesseract::task_composer_planning_nodes
         console_bridge::console_bridge
         tesseract::collision
  PRIVATE ${PCL_LIBRARIES})

target_include_directories(robot_planner PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC tesseract::environment
         tesseract::motion_planners
         tesseract::motion_planners_ompl
         tesseract::motion_planners_trajopt
         tesseract::motion_planners_trajopt_ifopt
         trajopt::trajopt_sqp
         trajopt::trajopt_ifopt
         tesseract::task_composer
         tesseract::task_composer_planning_nodes
         console_bridge::console_bridge
  PRIVATE ${PCL_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>"
                                           PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS})

macro(add_example example_name example_file)
  add_executable(${example_name} ${example_file})
  target_link_libraries(${example_name} PRIVATE ${PROJECT_NAME})
  target_include_directories(${example_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endmacro()

add_example(${PROJECT_NAME}_basic_cartesian_example src/basic_cartesian_example_node.cpp)
add_example(${PROJECT_NAME}_car_seat_example src/car_seat_example_node.cpp)
add_example(${PROJECT_NAME}_freespace_hybrid_example src/freespace_hybrid_example_node.cpp)
add_example(${PROJECT_NAME}_freespace_ompl_example src/freespace_ompl_example_node.cpp)
add_example(${PROJECT_NAME}_glass_upright_example src/glass_upright_example_node.cpp)
add_example(${PROJECT_NAME}_online_planning_example src/online_planning_example_node.cpp)
add_example(${PROJECT_NAME}_pick_and_place_example src/pick_and_place_example_node.cpp)
add_example(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example src/puzzle_piece_auxillary_axes_example_node.cpp)
add_example(${PROJECT_NAME}_puzzle_piece_example src/puzzle_piece_example_node.cpp)
add_example(${PROJECT_NAME}_scene_graph_example src/scene_graph_example_node.cpp)
add_example(${PROJECT_NAME}_benchmarks src/example_benchmarks_node.cpp)

enable_testing()
include(GoogleTest)
set(TESSERACT_TEST_ENV
    "TESSERACT_RESOURCE_PATH=${TESSERACT_TEST_SHARE_DIR}"
    "PATH=${TESSERACT_VCPKG_PREFIX}/bin;$ENV{PATH}")

function(add_tesseract_example_unit unit_name unit_file)
  add_executable(${unit_name} ${unit_file})
  target_link_libraries(${unit_name} PRIVATE ${PROJECT_NAME} GTest::gtest)
  target_include_directories(${unit_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  gtest_discover_tests(${unit_name} DISCOVERY_MODE PRE_TEST PROPERTIES ENVIRONMENT "${TESSERACT_TEST_ENV}")
endfunction()

add_tesseract_example_unit(examples_basic_cartesian_example_unit tests/examples/basic_cartesian_example_unit.cpp)
add_tesseract_example_unit(examples_car_seat_example_unit tests/examples/car_seat_example_unit.cpp)
add_tesseract_example_unit(examples_freespace_hybrid_example_unit tests/examples/freespace_hybrid_example_unit.cpp)
add_tesseract_example_unit(examples_glass_upright_example_unit tests/examples/glass_upright_example_unit.cpp)
add_tesseract_example_unit(examples_pick_and_place_example_unit tests/examples/pick_and_place_example_unit.cpp)
add_tesseract_example_unit(examples_puzzle_piece_auxillary_axes_example_unit tests/examples/puzzle_piece_auxillary_axes_example_unit.cpp)
add_tesseract_example_unit(examples_puzzle_piece_example_unit tests/examples/puzzle_piece_example_unit.cpp)
add_tesseract_example_unit(examples_scene_graph_example_unit tests/examples/scene_graph_example_unit.cpp)

add_executable(test_robot_planner src/test_robot_planner.cpp)
target_link_libraries(test_robot_planner PRIVATE robot_planner)
target_include_directories(test_robot_planner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME test_robot_planner COMMAND test_robot_planner)
set_tests_properties(test_robot_planner PROPERTIES ENVIRONMENT "${TESSERACT_TEST_ENV}")

# Fanuc R-2000iC/165F VAMP SIMD acceleration module
add_subdirectory(vamp_r2000ic)

# if (MSVC)
#   # 复制tesseract::tesseract_common 所在文件夹下的所有dll
#   add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#     COMMAND powershell -NoProfile -Command "Copy-Item -Path '$<TARGET_FILE_DIR:tesseract::tesseract_common>/*.dll' -Destination '$<TARGET_FILE_DIR:${PROJECT_NAME}>' -Force"
#     COMMENT "Copying Tesseract DLLs to output directory..."
#   )
#   add_custom_command(
#     TARGET ${PROJECT_NAME} POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy
#         "$<TARGET_RUNTIME_DLLS:${PROJECT_NAME}>"   # 自动获取所有依赖 DLL 路径
#         "$<TARGET_FILE_DIR:${PROJECT_NAME}>"       # 目标文件输出目录
#     COMMAND_EXPAND_LISTS                   # 必须，用于展开 DLL 路径列表
#   )
# endif (MSVC)
