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
# add_compile_options(-fsanitize=address)
# add_link_options(-fsanitize=address)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(OpenCV REQUIRED)

set(ROS2_DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "event_camera_msgs"
  "event_camera_codecs"
  "rosbag2_storage"
  "rosbag2_cpp"
  "sensor_msgs")

foreach(pkg ${ROS2_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

ament_auto_find_build_dependencies(REQUIRED ${ROS2_DEPENDENCIES})

if(${rosbag2_storage_VERSION} VERSION_GREATER_EQUAL "0.26.1")
  add_definitions(-DUSE_ROSBAG2_STORAGE_RECV_TIME)
endif()

# --------- sync test
ament_auto_add_executable(sync_test  src/sync_test_ros2.cpp)

# -------- echo tool
ament_auto_add_executable(echo  src/echo_ros2.cpp)

# -------- event_rate tool
ament_auto_add_executable(event_rate  src/event_rate.cpp)

# -------- performance tool
ament_auto_add_executable(perf  src/perf_ros2.cpp)

# -------- conversion tools
ament_auto_add_executable(bag_to_raw src/bag_to_raw_ros2.cpp)
ament_auto_add_executable(raw_to_bag src/raw_to_bag.cpp)

# -------- movie maker
ament_auto_add_executable(movie_maker  src/movie_maker_ros2.cpp)
target_link_libraries(movie_maker opencv_core opencv_imgcodecs)

# -------- event statistics
ament_auto_add_executable(event_statistics  src/event_statistics.cpp)

# -------- plot events
ament_auto_add_executable(plot_events  src/plot_events_ros2.cpp)

# -------- find_trigger_events
ament_auto_add_executable(find_trigger_events  src/find_trigger_events_ros2.cpp)

# -------- find_trigger_events
ament_auto_add_executable(trigger_delay  src/trigger_delay.cpp)

# the nodes must go into the paroject specific lib directory or else
# the launch file will not find it

install(TARGETS
  perf
  echo
  event_rate
  sync_test
  bag_to_raw
  raw_to_bag
  movie_maker
  plot_events
  event_statistics
  find_trigger_events
  trigger_delay
  DESTINATION lib/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_black REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_black()
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
