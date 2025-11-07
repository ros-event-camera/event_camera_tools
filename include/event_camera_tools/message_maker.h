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

#ifndef EVENT_CAMERA_TOOLS__MESSAGE_MAKER_H_
#define EVENT_CAMERA_TOOLS__MESSAGE_MAKER_H_

#include <event_camera_codecs/encoder.h>
#include <event_camera_codecs/event_processor.h>
#include <event_camera_tools/check_endian.h>
#include <inttypes.h>

#include <chrono>

#ifdef USING_ROS_1
#include <event_camera_msgs/EventPacket.h>
#include <ros/ros.h>
#else
#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

namespace event_camera_tools
{
using event_camera_codecs::Decoder;
using event_camera_codecs::Encoder;
#ifdef USING_ROS_1
using event_camera_msgs::EventPacket;
typedef ros::Time RosTimeType;
#define GENERIC_ROS_DURATION(X) (ros::Duration().fromNSec(X))
#define ROS_MSG_PTR Ptr
#else
using event_camera_msgs::msg::EventPacket;
typedef rclcpp::Time RosTimeType;
#define ROS_MSG_PTR UniquePtr
#define GENERIC_ROS_DURATION(X) (rclcpp::Duration(std::chrono::nanoseconds(X)))
#endif

template <typename MsgType>
class MessageMaker : public event_camera_codecs::EventProcessor
{
  using Encoder = event_camera_codecs::Encoder;

public:
  explicit MessageMaker(const std::string & codec)
  {
    eventEncoder_ = Encoder::newInstance(codec);
    triggerEncoder_ = Encoder::newInstance(codec);
  }
  // ---------- from the EventProcessor interface:
  void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    if (waitForSensorTime_) {
      waitForSensorTime_ = false;
      baseSensorTime_ = sensor_time;
    }
    if (eventMsg_) {
      typename decltype(MsgType::events)::value_type e;
      e.ts = messageRosStamp_ + GENERIC_ROS_DURATION(sensor_time - baseSensorTime_);
      e.x = ex;
      e.y = ey;
      e.polarity = polarity;
      eventMsg_->events.push_back(e);
    }
  }

  bool eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t id) override
  {
    if (waitForSensorTime_) {
      waitForSensorTime_ = false;
      baseSensorTime_ = sensor_time;
    }
    if (triggerMsg_) {
      typename decltype(MsgType::events)::value_type e;
      e.ts = messageRosStamp_ + GENERIC_ROS_DURATION(sensor_time - baseSensorTime_);
      e.x = id;
      e.y = 0;
      e.polarity = edge;
      triggerMsg_->events.push_back(e);
    }
    return (true);
  }

  void finished() override {}
  void rawData(const char *, size_t) override {}
  // -------- end of inherited

  void initializeEventMessage(uint32_t width, uint32_t height, const decltype(MsgType::header) & h)
  {
    if (!eventMsg_) {
      eventMsg_.reset(new MsgType());
      eventMsg_->width = width;
      eventMsg_->height = height;
      initializeMoreEvent();
    }
    eventMsg_->header = h;
  }

  void initializeTriggerMessage(
    uint32_t width, uint32_t height, const decltype(MsgType::header) & h)
  {
    if (!triggerMsg_) {
      triggerMsg_.reset(new MsgType());
      triggerMsg_->width = width;
      triggerMsg_->height = height;
      initializeMoreTrigger();
    }
    triggerMsg_->header = h;
  }

  void initializeMoreEvent()
  {
    // -- no-op for dvs & proph msgs
  }
  void initializeMoreTrigger()
  {
    // -- no-op for dvs & proph msgs
  }

  void setRosStamp(const RosTimeType & t)
  {
    messageRosStamp_ = t;
    if (eventMsg_) {
      eventMsg_->header.stamp = t;
    }
    if (triggerMsg_) {
      triggerMsg_->header.stamp = t;
    }
    waitForSensorTime_ = true;  // updated ros time, must wait for first time stamp
  }

  typename MsgType::ROS_MSG_PTR resetEventMessage()
  {
    maxEventSize_ = std::max(maxEventSize_, eventMsg_->events.size());
#ifdef USING_ROS_1
    eventMsg_->header.seq = eventSeq_++;
#endif
    return (std::move(eventMsg_));
  }

  typename MsgType::ROS_MSG_PTR resetTriggerMessage()
  {
    maxTriggerSize_ = std::max(maxTriggerSize_, triggerMsg_->events.size());
#ifdef USING_ROS_1
    triggerMsg_->header.seq = triggerSeq_++;
#endif
    return (std::move(triggerMsg_));
  }

private:
  // ------------ variables
  RosTimeType messageRosStamp_;
  uint64_t baseSensorTime_{0};
  bool isBaseSensorTimeValid_{false};
  bool waitForSensorTime_{true};
  typename MsgType::ROS_MSG_PTR eventMsg_;
  typename MsgType::ROS_MSG_PTR triggerMsg_;
  std::shared_ptr<Encoder> eventEncoder_;
  std::shared_ptr<Encoder> triggerEncoder_;
  size_t maxEventSize_{0};
  size_t maxTriggerSize_{0};
  bool isInitialized_{false};
  uint64_t eventSeq_{0};
  uint64_t triggerSeq_{0};
};

template <>
void MessageMaker<EventPacket>::eventCD(
  uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity)
{
  if (waitForSensorTime_) {
    waitForSensorTime_ = false;
    baseSensorTime_ = sensor_time;
  }
  if (eventMsg_) {
    const uint32_t dt = static_cast<uint32_t>(sensor_time - baseSensorTime_);
    eventEncoder_->encodeCD(dt, ex, ey, polarity);
  }
}

template <>
bool MessageMaker<EventPacket>::eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t id)
{
  if (waitForSensorTime_) {
    waitForSensorTime_ = false;
    baseSensorTime_ = sensor_time;
  }
  if (triggerMsg_) {
    const uint32_t dt = static_cast<uint32_t>(sensor_time - baseSensorTime_);
    triggerEncoder_->encodeExtTrigger(dt, edge, id);
  }
  return (true);
}

template <>
void MessageMaker<EventPacket>::initializeMoreEvent()
{
  eventEncoder_->setBuffer(&(eventMsg_->events));
  eventMsg_->is_bigendian = check_endian::isBigEndian();
  eventMsg_->encoding = "mono";
}

template <>
void MessageMaker<EventPacket>::initializeMoreTrigger()
{
  triggerEncoder_->setBuffer(&(triggerMsg_->events));
  triggerMsg_->is_bigendian = check_endian::isBigEndian();
  triggerMsg_->encoding = "trigger";
}

template <>
EventPacket::ROS_MSG_PTR MessageMaker<EventPacket>::resetEventMessage()
{
  if (eventMsg_->events.empty()) {
    return (0);
  }
  maxEventSize_ = std::max(maxEventSize_, eventMsg_->events.size());
  eventMsg_->time_base = baseSensorTime_;
#ifdef USING_ROS_1
  eventMsg_->header.seq = eventSeq_;
#endif
  eventMsg_->seq = eventSeq_;
  eventSeq_++;
  if (eventEncoder_) {
    eventEncoder_->flush();
  }
  return (std::move(eventMsg_));
}

template <>
EventPacket::ROS_MSG_PTR MessageMaker<EventPacket>::resetTriggerMessage()
{
  if (triggerMsg_->events.empty()) {
    return (0);
  }
  maxTriggerSize_ = std::max(maxTriggerSize_, triggerMsg_->events.size());
  triggerMsg_->time_base = baseSensorTime_;
#ifdef USING_ROS_1
  triggerMsg_->header.seq = triggerSeq_;
#endif
  triggerMsg_->seq = triggerSeq_;
  triggerSeq_++;
  if (triggerEncoder_) {
    triggerEncoder_->flush();
  }
  return (std::move(triggerMsg_));
}

}  // namespace event_camera_tools
#endif  // EVENT_CAMERA_TOOLS__MESSAGE_MAKER_H_
