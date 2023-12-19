#
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# the rosbag api changed between distros
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "foxy" OR
      $ENV{ROS_DISTRO} STREQUAL "galactic")
    add_definitions(-DUSE_OLD_ROSBAG_API)
  endif()
else()
  message(ERROR "ROS_DISTRO environment variable is not set!")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(OpenCV REQUIRED)

set(ROS2_DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "dvs_msgs"
  "prophesee_event_msgs"
  "event_camera_msgs"
  "event_camera_codecs"
  "rosbag2_cpp")

foreach(pkg ${ROS2_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

ament_auto_find_build_dependencies(REQUIRED ${ROS2_DEPENDENCIES})


# --------- sync test
ament_auto_add_executable(sync_test  src/sync_test_ros2.cpp)

# -------- echo tool
ament_auto_add_executable(echo  src/echo_ros2.cpp)

# -------- performance tool
ament_auto_add_executable(perf  src/perf_ros2.cpp)

# -------- conversion tools
ament_auto_add_executable(bag_to_raw src/bag_to_raw_ros2.cpp)
ament_auto_add_executable(raw_to_bag src/raw_to_bag_ros2.cpp)

# -------- movie maker
ament_auto_add_executable(movie_maker  src/movie_maker_ros2.cpp)
target_link_libraries(movie_maker opencv_core opencv_imgcodecs)

# -------- plot events
ament_auto_add_executable(plot_events  src/plot_events_ros2.cpp)

# -------- find_trigger_events
ament_auto_add_executable(find_trigger_events  src/find_trigger_events_ros2.cpp)

# -------- republish node and composable
ament_auto_add_library(republish SHARED src/republish_composable.cpp)
target_include_directories(republish PRIVATE include)
rclcpp_components_register_nodes(republish "event_camera_tools::RepublishComposable")

ament_auto_add_executable(republish_node  src/republish_node_ros2.cpp)


# the node must go into the paroject specific lib directory or else
# the launch file will not find it

install(TARGETS
  perf
  echo
  sync_test
  bag_to_raw
  raw_to_bag
  movie_maker
  plot_events
  find_trigger_events
  republish_node
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  republish
  DESTINATION lib)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()
endif()

ament_package()
