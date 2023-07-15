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
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>

#include <memory>

#include "event_camera_tools/republish_ros1.h"

int main(int argc, char ** argv)
{
  std::string topic;
  ros::init(argc, argv, "republish_node");
  ros::NodeHandle pnh("~");

  const std::string type = pnh.param<std::string>("output_message_type", "event_packet");
  ROS_INFO_STREAM("republishing as message type: " << type);
  if (type == "dvs") {
    event_camera_tools::Republish<dvs_msgs::EventArray> node(pnh);
    ros::spin();
  } else if (type == "prophesee") {
    event_camera_tools::Republish<prophesee_event_msgs::EventArray> node(pnh);
    ros::spin();
  } else if (type == "event_packet") {
    event_camera_tools::Republish<event_camera_msgs::EventPacket> node(pnh);
    ros::spin();
  } else {
    ROS_ERROR_STREAM("invalid message type: " << type);
    throw(std::runtime_error("invalid message type"));
  }
  return 0;
}
