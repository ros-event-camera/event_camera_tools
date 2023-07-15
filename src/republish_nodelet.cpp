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

#include <dvs_msgs/EventArray.h>
#include <event_camera_msgs/EventPacket.h>
#include <nodelet/nodelet.h>
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>

#include <memory>

#include "event_camera_tools/republish_ros1.h"

namespace event_camera_tools
{
class RepublishNodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    nh_ = getPrivateNodeHandle();
    const std::string type = nh_.param<std::string>("output_message_type", "event_packet");
    ROS_INFO_STREAM("republishing as message type: " << type);
    if (type == "dvs") {
      nodeDvs_ = std::make_shared<Republish<dvs_msgs::EventArray>>(nh_);
    } else if (type == "prophesee") {
      nodeProphesee_ = std::make_shared<Republish<prophesee_event_msgs::EventArray>>(nh_);
    } else if (type == "event_packet") {
      nodeEventPacket_ = std::make_shared<Republish<event_camera_msgs::EventPacket>>(nh_);
    } else {
      ROS_ERROR_STREAM("invalid message type: " << type);
      throw(std::runtime_error("invalid message type"));
    }
  }

private:
  // ------ variables --------
  ros::NodeHandle nh_;
  std::shared_ptr<Republish<dvs_msgs::EventArray>> nodeDvs_;
  std::shared_ptr<Republish<prophesee_event_msgs::EventArray>> nodeProphesee_;
  std::shared_ptr<Republish<event_camera_msgs::EventPacket>> nodeEventPacket_;
};
}  // namespace event_camera_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(event_camera_tools::RepublishNodelet, nodelet::Nodelet)
