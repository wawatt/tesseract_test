cmake_minimum_required(VERSION 3.22)
project(robot_sim_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL)
find_package(unofficial-osg CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)

# Find osgQOpenGL library
find_library(OSGQOPENGL_LIBRARY
  NAMES osgQOpenGL
  PATHS "${VCPKG_INSTALLED_DIR}/x64-windows/lib"
  REQUIRED
)

find_path(OSGQOPENGL_INCLUDE_DIR
  NAMES osgQOpenGL/osgQOpenGLWidget
  PATHS "${VCPKG_INSTALLED_DIR}/x64-windows/include"
  REQUIRED
)

set(APP_SOURCES
  src/main.cpp
  src/main_window.cpp
  src/osg_viewer_widget.cpp
  src/robot_visual_node.cpp
  src/mesh_loader.cpp
  src/simulation_controller.cpp
  src/jog_panel_dock.cpp
  src/scene_tree_dock.cpp
  src/planning_dock.cpp
  src/timeline_dock.cpp
  src/log_dock.cpp
  src/add_obstacle_dialog.cpp
  resources/resources.qrc
)

set(APP_HEADERS
  include/main_window.h
  include/osg_viewer_widget.h
  include/robot_visual_node.h
  include/mesh_loader.h
  include/simulation_controller.h
  include/jog_panel_dock.h
  include/scene_tree_dock.h
  include/planning_dock.h
  include/timeline_dock.h
  include/log_dock.h
  include/add_obstacle_dialog.h
)

add_executable(robot_sim_app ${APP_SOURCES} ${APP_HEADERS})

target_include_directories(robot_sim_app PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${PROJECT_SOURCE_DIR}/include"
  "${OSGQOPENGL_INCLUDE_DIR}"
)

target_link_libraries(robot_sim_app PRIVATE
  robot_planner
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::OpenGL
  unofficial::osg::osg
  unofficial::osg::osgViewer
  unofficial::osg::osgDB
  unofficial::osg::osgGA
  unofficial::osg::osgUtil
  unofficial::osg::osgText
  assimp::assimp
  ${OSGQOPENGL_LIBRARY}
)

set_target_properties(robot_sim_app PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/workspace/Release"
  RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/workspace/Debug"
)

# Copy Qt5 and OSG required DLLs to workspace/Release post build
if (WIN32)
  add_custom_command(TARGET robot_sim_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:robot_sim_app>/platforms"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${VCPKG_INSTALLED_DIR}/x64-windows/plugins/platforms/qwindows.dll"
      "$<TARGET_FILE_DIR:robot_sim_app>/platforms/"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/Qt5Core.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/Qt5Gui.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/Qt5Widgets.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/Qt5OpenGL.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/osg.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/osgViewer.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/osgDB.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/osgGA.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/osgUtil.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/osgText.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/OpenThreads.dll"
      "${VCPKG_INSTALLED_DIR}/x64-windows/bin/osg145-osgQOpenGL.dll"
      "$<TARGET_FILE_DIR:robot_sim_app>"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${VCPKG_INSTALLED_DIR}/x64-windows/plugins/osgPlugins-3.6.5"
      "$<TARGET_FILE_DIR:robot_sim_app>/osgPlugins-3.6.5"
    COMMENT "Deploying Qt5 and OSG runtime DLLs and plugins to output directory..."
  )
endif()

