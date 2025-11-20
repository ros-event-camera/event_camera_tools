// -*-c++-*--------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EVENT_CAMERA_TOOLS__FRAME_TIME_H_
#define EVENT_CAMERA_TOOLS__FRAME_TIME_H_

#include <inttypes.h>

#include <rclcpp/rclcpp.hpp>

namespace event_camera_tools
{
struct FrameTime
{
  explicit FrameTime(rclcpp::Time rt, uint64_t st) : ros_time(rt), sensor_time(st) {}
  rclcpp::Time ros_time;
  uint64_t sensor_time{0};
};
}  // namespace event_camera_tools
#endif  // EVENT_CAMERA_TOOLS__FRAME_TIME_H_
