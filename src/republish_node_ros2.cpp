// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <dvs_msgs/msg/event_array.hpp>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <memory>
#include <prophesee_event_msgs/msg/event_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "event_camera_tools/republish_ros2.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string type;
  auto node = std::make_shared<rclcpp::Node>(
    "republish", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  node->get_parameter_or("output_message_type", type, std::string("event_packet"));
  if (type == "dvs") {
    event_camera_tools::Republish<dvs_msgs::msg::EventArray> rep(node.get());
    rclcpp::spin(node);  // should not return
  } else if (type == "prophesee") {
    event_camera_tools::Republish<prophesee_event_msgs::msg::EventArray> rep(node.get());
    rclcpp::spin(node);  // should not return
  } else if (type == "event_packet") {
    event_camera_tools::Republish<event_camera_msgs::msg::EventPacket> rep(node.get());
    rclcpp::spin(node);  // should not return
  } else {
    std::cerr << "invalid message type: " << type << std::endl;
    throw(std::runtime_error("invalid message type"));
  }
  rclcpp::shutdown();
  return 0;
}
