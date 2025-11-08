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

add_definitions(-DUSING_ROS_1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  event_camera_msgs
  event_camera_codecs
  rosbag)

find_package(OpenCV REQUIRED)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS})

# --------- sync test
add_executable(sync_test src/sync_test_ros1.cpp)
target_link_libraries(sync_test ${catkin_LIBRARIES})

# --------- echo tool
add_executable(echo src/echo_ros1.cpp)
target_link_libraries(echo ${catkin_LIBRARIES})

# --------- performance tool
add_executable(perf src/perf_ros1.cpp)
target_link_libraries(perf ${catkin_LIBRARIES})

# --------- conversion tools

add_executable(raw_to_bag src/raw_to_bag.cpp)
target_link_libraries(raw_to_bag ${catkin_LIBRARIES})

add_executable(bag_to_raw src/bag_to_raw_ros1.cpp)
target_link_libraries(bag_to_raw ${catkin_LIBRARIES})

# --------- movie maker
add_executable(movie_maker src/movie_maker_ros1.cpp)
target_link_libraries(movie_maker ${catkin_LIBRARIES} opencv_core opencv_imgcodecs)


#############
## Install ##
#############

install(TARGETS echo perf bag_to_raw movie_maker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#############
## Testing ##
#############

# To be done...
