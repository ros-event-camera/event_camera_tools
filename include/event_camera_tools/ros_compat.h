// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef EVENT_CAMERA_TOOLS__ROS_COMPAT_H_
#define EVENT_CAMERA_TOOLS__ROS_COMPAT_H_

#ifdef USING_ROS_1
#include <event_camera_msgs/EventPacket.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#else
#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#endif

namespace ros_compat
{
#ifdef USING_ROS_1
//
// ------------------ ROS1 ---------------------
//
using Time = ros::Time;
using Duration = ros::Duration;
using Writer = rosbag::Bag;
Time now() { return (ros::Time::now()); }
Time time_from_sec(const double sec) { return (Time(sec)); }

uint64_t to_nanoseconds(const Time & t) { return (t.toNSec()); }
Duration duration_from_nanoseconds(uint64_t nsec) { return (Duration().fromNSec(nsec)); }

#else
//
// ------------------ ROS2 ---------------------
//
using Time = rclcpp::Time;
using Duration = rclcpp::Duration;
using Writer = rosbag2_cpp::Writer;

Time now() { return (rclcpp::Clock().now()); }
Time time_from_sec(const double sec) { return (Time(static_cast<uint64_t>(sec * 1e9))); }
uint64_t to_nanoseconds(const Time & t) { return (t.nanoseconds()); }
Duration duration_from_nanoseconds(uint64_t nsec) { return (Duration::from_nanoseconds(nsec)); }
#endif
}  // namespace ros_compat
#endif  // EVENT_CAMERA_TOOLS__ROS_COMPAT_H_
