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
#include <rclcpp_components/register_node_macro.hpp>

#include "event_camera_tools/republish_ros2.h"

namespace event_camera_tools
{
class RepublishComposable : public rclcpp::Node
{
public:
  explicit RepublishComposable(const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<event_camera_tools::Republish<dvs_msgs::msg::EventArray>> nodeDvs_;
  std::shared_ptr<event_camera_tools::Republish<prophesee_event_msgs::msg::EventArray>>
    nodeProphesee_;
  std::shared_ptr<event_camera_tools::Republish<event_camera_msgs::msg::EventPacket>>
    nodeEventPacket_;
};

RepublishComposable::RepublishComposable(const rclcpp::NodeOptions & options)
: rclcpp::Node("republish", options)
{
  std::string type;
  this->get_parameter_or("output_message_type", type, std::string("event_packet"));
  if (type == "dvs") {
    nodeDvs_ = std::make_shared<Republish<dvs_msgs::msg::EventArray>>(this);
  } else if (type == "prophesee") {
    nodeProphesee_ = std::make_shared<Republish<prophesee_event_msgs::msg::EventArray>>(this);
  } else if (type == "event_packet") {
    nodeEventPacket_ = std::make_shared<Republish<event_camera_msgs::msg::EventPacket>>(this);
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "invalid message type: " << type);
    throw(std::runtime_error("invalid message type"));
  }
}

}  // namespace event_camera_tools
RCLCPP_COMPONENTS_REGISTER_NODE(event_camera_tools::RepublishComposable)
