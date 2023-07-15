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

#ifndef EVENT_CAMERA_TOOLS__REPUBLISH_ROS2_H_
#define EVENT_CAMERA_TOOLS__REPUBLISH_ROS2_H_

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>
#include <event_camera_tools/check_endian.h>
#include <event_camera_tools/message_maker.h>
#include <inttypes.h>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

namespace event_camera_tools
{
using event_camera_codecs::Decoder;
using event_camera_codecs::EventPacket;

template <typename MsgType>
class Republish
{
public:
  explicit Republish(rclcpp::Node * node) : nh_(node), messageMaker_("mono")
  {
    int qs;
    nh_->get_parameter_or("recv_queue_size", qs, 1000);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qs)).best_effort().durability_volatile();
    sub_ = nh_->create_subscription<EventPacket>(
      "~/input_events", qos, std::bind(&Republish::eventMsg, this, std::placeholders::_1));
    nh_->get_parameter_or("send_queue_size", qs, 1000);
    eventPub_ = nh_->create_publisher<MsgType>(
      "~/output_events", rclcpp::QoS(rclcpp::KeepLast(qs)).best_effort().durability_volatile());
    triggerPub_ = nh_->create_publisher<MsgType>(
      "~/output_triggers", rclcpp::QoS(rclcpp::KeepLast(qs)).best_effort().durability_volatile());
  }
  Republish(const Republish &) = delete;
  Republish & operator=(const Republish &) = delete;

  inline void decode(const EventPacket::ConstSharedPtr & msg)
  {
    auto decoder = decoderFactory_.getInstance(msg->encoding, msg->width, msg->height);
    if (!decoder) {
      printf("unsupported encoding: %s\n", msg->encoding.c_str());
      return;
    }
    decoder->decode(&msg->events[0], msg->events.size(), &messageMaker_);
  }

  void eventMsg(EventPacket::ConstSharedPtr msg)
  {
    const bool pubEvents = eventPub_->get_subscription_count();
    const bool pubTriggers = triggerPub_->get_subscription_count();
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
      typename MsgType::UniquePtr msg = messageMaker_.resetEventMessage();
      if (msg) {
        eventPub_->publish(std::move(msg));
      }
    }
    if (pubTriggers) {
      typename MsgType::UniquePtr msg = messageMaker_.resetTriggerMessage();
      if (msg) {
        triggerPub_->publish(std::move(msg));
      }
    }
  }
  // ---------- variables
  rclcpp::Node * nh_;
  rclcpp::Subscription<EventPacket>::SharedPtr sub_;
  typename rclcpp::Publisher<MsgType>::SharedPtr eventPub_;
  typename rclcpp::Publisher<MsgType>::SharedPtr triggerPub_;
  event_camera_codecs::DecoderFactory<EventPacket, MessageMaker<MsgType>> decoderFactory_;
  MessageMaker<MsgType> messageMaker_;
};
}  // namespace event_camera_tools
#endif  // EVENT_CAMERA_TOOLS__REPUBLISH_ROS2_H_
