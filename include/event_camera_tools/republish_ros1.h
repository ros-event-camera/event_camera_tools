// -*-c++-*--------------------------------------------------------------------
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

#ifndef EVENT_CAMERA_TOOLS__REPUBLISH_ROS1_H_
#define EVENT_CAMERA_TOOLS__REPUBLISH_ROS1_H_

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>
#include <event_camera_msgs/EventPacket.h>
#include <event_camera_tools/check_endian.h>
#include <event_camera_tools/message_maker.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <unordered_map>

namespace event_camera_tools
{
using event_camera_codecs::Decoder;
using event_camera_codecs::EventPacket;

template <typename MsgType>
class Republish
{
public:
  explicit Republish(const ros::NodeHandle & nh) : nh_(nh), messageMaker_("mono")
  {
    sub_ = nh_.subscribe(
      "input_events", nh_.param<int>("recv_queue_size", 1000), &Republish::eventMsg, this);
    eventPub_ = nh_.advertise<MsgType>("output_events", nh_.param<int>("send_queue_size", 1000));
    triggerPub_ =
      nh_.advertise<MsgType>("output_triggers", nh_.param<int>("send_queue_size", 1000));
  }
  Republish(const Republish &) = delete;
  Republish & operator=(const Republish &) = delete;

  inline void decode(const EventPacket::ConstPtr & msg)
  {
    auto decoder = decoderFactory_.getInstance(msg->encoding, msg->width, msg->height);
    if (!decoder) {
      printf("unsupported encoding: %s\n", msg->encoding.c_str());
      return;
    }
    decoder->decode(&msg->events[0], msg->events.size(), &messageMaker_);
  }

  void eventMsg(EventPacket::ConstPtr msg)
  {
    const bool pubEvents = eventPub_.getNumSubscribers();
    const bool pubTriggers = triggerPub_.getNumSubscribers();
    if (pubEvents) {
      messageMaker_.initializeEventMessage(msg->width, msg->height, msg->header);
    }
    if (pubTriggers) {
      messageMaker_.initializeTriggerMessage(msg->width, msg->height, msg->header);
    }
    if (pubEvents || pubTriggers) {
      messageMaker_.setRosStamp(msg->header.stamp);
      // decode message, produces callbacks into message maker
      decode(msg);
    }
    if (pubEvents) {
      auto msg = messageMaker_.resetEventMessage();
      if (msg) {
        eventPub_.publish(std::move(msg));
      }
    }
    if (pubTriggers) {
      auto msg = messageMaker_.resetTriggerMessage();
      if (msg) {
        triggerPub_.publish(std::move(msg));
      }
    }
  }
  // ---------- variables
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher eventPub_;
  ros::Publisher triggerPub_;

  event_camera_codecs::DecoderFactory<EventPacket, MessageMaker<MsgType>> decoderFactory_;
  MessageMaker<MsgType> messageMaker_;
};
}  // namespace event_camera_tools
#endif  // EVENT_CAMERA_TOOLS__REPUBLISH_ROS1_H_
