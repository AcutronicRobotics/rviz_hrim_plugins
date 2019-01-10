cmake_minimum_required(VERSION 3.5)

project(rviz_hrim_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Tests currently only run on OS X @ OSRF jenkins
# Enable on Linux by providing a display, enable on Windows via EnableDisplayTests=True
option(EnableDisplayTests "EnableDisplayTests")
set(DisplayTests "False" CACHE STRING "DisplayTestsVariable")

if(DEFINED ENV{DISPLAY})
  set(DISPLAYPRESENT TRUE)
endif()

if(APPLE OR DISPLAYPRESENT OR EnableDisplayTests STREQUAL "True")
  message(STATUS "Enabling tests requiring a display")
else()
  set(SKIP_DISPLAY_TESTS "SKIP_TEST")
endif()

# options and directories for visual tests (see visual_testing_framework documentation)
option(EnableVisualTests "decides whether or not to enable the tests")

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_images)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_images)

# Run visual tests only if "EnableVisualTests=True"
if(EnableVisualTests STREQUAL "True")
  message(STATUS "Enabling visual tests")
else()
  set(SKIP_VISUAL_TESTS "SKIP_TEST")
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_visual_testing_framework REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hrim_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)  # provided by tinyxml_vendor
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(hrim_sensor_camera_msgs REQUIRED)
find_package(hrim_sensor_imu_msgs REQUIRED)
find_package(hrim_sensor_rangefinder_msgs REQUIRED)
find_package(hrim_sensor_3dcameratof_msgs REQUIRED)

# These need to be added in the add_library() call so AUTOMOC detects them.
set(rviz_hrim_plugins_headers_to_moc
  include/rviz_hrim_plugins/displays/image/image_display.hpp
  include/rviz_hrim_plugins/displays/camera/camera_display.hpp
  include/rviz_hrim_plugins/displays/imu/imu_visual.hpp
  include/rviz_hrim_plugins/displays/imu/imu_display.hpp
  include/rviz_hrim_plugins/displays/range/range_display.hpp
  include/rviz_hrim_plugins/displays/pointcloud/point_cloud_common.hpp
  include/rviz_hrim_plugins/displays/pointcloud/point_cloud_display.hpp
  include/rviz_hrim_plugins/displays/pointcloud/point_cloud2_display.hpp
  include/rviz_hrim_plugins/displays/pointcloud/point_cloud_transformer.hpp
  include/rviz_hrim_plugins/displays/pointcloud/point_cloud_transformer_factory.hpp
  include/rviz_hrim_plugins/displays/pointcloud/point_cloud_helpers.hpp
  include/rviz_hrim_plugins/displays/pointcloud/transformers/axis_color_pc_transformer.hpp
  include/rviz_hrim_plugins/displays/pointcloud/transformers/flat_color_pc_transformer.hpp
  include/rviz_hrim_plugins/displays/pointcloud/transformers/intensity_pc_transformer.hpp
  include/rviz_hrim_plugins/displays/pointcloud/transformers/rgb8_pc_transformer.hpp
  include/rviz_hrim_plugins/displays/pointcloud/transformers/rgbf32_pc_transformer.hpp
)

set(rviz_hrim_plugins_source_files
  src/rviz_hrim_plugins/displays/image/image_display.cpp
  src/rviz_hrim_plugins/displays/image/ros_image_texture.cpp
  src/rviz_hrim_plugins/displays/camera/camera_display.cpp
  src/rviz_hrim_plugins/displays/imu/imu_visual.cpp
  src/rviz_hrim_plugins/displays/imu/imu_display.cpp
  src/rviz_hrim_plugins/displays/range/range_display.cpp
  src/rviz_hrim_plugins/displays/pointcloud/transformers/axis_color_pc_transformer.cpp
  src/rviz_hrim_plugins/displays/pointcloud/transformers/flat_color_pc_transformer.cpp
  src/rviz_hrim_plugins/displays/pointcloud/transformers/intensity_pc_transformer.cpp
  src/rviz_hrim_plugins/displays/pointcloud/transformers/rgb8_pc_transformer.cpp
  src/rviz_hrim_plugins/displays/pointcloud/transformers/rgbf32_pc_transformer.cpp
  src/rviz_hrim_plugins/displays/pointcloud/transformers/xyz_pc_transformer.cpp
  src/rviz_hrim_plugins/displays/pointcloud/point_cloud_common.cpp
  src/rviz_hrim_plugins/displays/pointcloud/point_cloud_to_point_cloud2.cpp
  src/rviz_hrim_plugins/displays/pointcloud/point_cloud_transformer_factory.cpp
  src/rviz_hrim_plugins/displays/pointcloud/point_cloud_selection_handler.cpp
  src/rviz_hrim_plugins/displays/pointcloud/point_cloud_display.cpp
  src/rviz_hrim_plugins/displays/pointcloud/point_cloud2_display.cpp
)

add_library(rviz_hrim_plugins SHARED
  ${rviz_hrim_plugins_headers_to_moc}
  ${rviz_hrim_plugins_source_files}
)

target_include_directories(rviz_hrim_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rviz_rendering_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${resource_retriever_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${hrim_sensor_camera_msgs_INCLUDE_DIRS}
  ${hrim_sensor_imu_msgs_INCLUDE_DIRS}
  ${hrim_sensor_rangefinder_msgs_INCLUDE_DIRS}
  ${hrim_sensor_3dcameratof_msgs_INCLUDE_DIRS}
)

target_link_libraries(rviz_hrim_plugins
  resource_retriever::resource_retriever
  rviz_common::rviz_common
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_hrim_plugins PRIVATE "RVIZ_HRIM_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(rviz_hrim_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(rviz_hrim_plugins
  hrim_geometry_msgs
  hrim_sensor_camera_msgs
  hrim_sensor_imu_msgs
  hrim_sensor_rangefinder_msgs
  hrim_sensor_3dcameratof_msgs
  std_msgs
  rclcpp
  resource_retriever
  urdf
  visualization_msgs
)

ament_export_include_directories(include)
ament_export_interfaces(rviz_hrim_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  hrim_geometry_msgs
  hrim_sensor_camera_msgs
  hrim_sensor_3dcameratof_msgs
  rclcpp
  urdf
  visualization_msgs
)

install(
  TARGETS rviz_hrim_plugins
  EXPORT rviz_hrim_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

#install(
#  DIRECTORY "${CMAKE_SOURCE_DIR}/icons"
#  DESTINATION "share/${PROJECT_NAME}"
#)

ament_package()
