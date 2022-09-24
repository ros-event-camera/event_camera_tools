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

#ifndef EVENT_ARRAY_TOOLS__REPUBLISH_ROS1_H_
#define EVENT_ARRAY_TOOLS__REPUBLISH_ROS1_H_

#include <event_array_codecs/decoder.h>
#include <event_array_codecs/encoder.h>
#include <event_array_codecs/event_processor.h>
#include <event_array_msgs/EventArray.h>
#include <event_array_tools/check_endian.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <unordered_map>

namespace event_array_tools
{
using event_array_codecs::Decoder;
using event_array_codecs::Encoder;
using event_array_msgs::EventArray;

template <typename MsgType>
class MessageMaker : public event_array_codecs::EventProcessor
{
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
    typename decltype(MsgType::events)::value_type e;
    e.ts = ros::Time().fromNSec(messageRosStamp_ + sensor_time - baseSensorTime_);
    e.x = ex;
    e.y = ey;
    e.polarity = polarity;
    eventMsg_->events.push_back(e);
  }

  void eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t id) override
  {
    if (waitForSensorTime_) {
      waitForSensorTime_ = false;
      baseSensorTime_ = sensor_time;
    }
    typename decltype(MsgType::events)::value_type e;
    e.ts = ros::Time().fromNSec(messageRosStamp_ + sensor_time - baseSensorTime_);
    e.x = id;
    e.y = 0;
    e.polarity = edge;
    triggerMsg_->events.push_back(e);
  }

  void finished() override {}
  void rawData(const char *, size_t) override {}
  // -------- end of inherited

  void messageDone()
  {
    if (!eventMsg_->events.empty()) {
      eventMsg_->header.seq = eventSeq_++;
    }
    if (!triggerMsg_->events.empty()) {
      triggerMsg_->header.seq = triggerSeq_++;
    }
  }

  void initialize(uint32_t width, uint32_t height, const std_msgs::Header & h)
  {
    eventMsg_.reset(new MsgType());
    eventMsg_->width = width;
    eventMsg_->height = height;
    eventMsg_->header = h;
    triggerMsg_.reset(new MsgType());
    triggerMsg_->width = width;
    triggerMsg_->height = height;
    triggerMsg_->header = h;
    initializeMore();
    isInitialized_ = true;
  }
  void initializeMore()
  {
    // -- no-op for dvs & proph msgs
  }
  bool isInitialized() const { return (isInitialized_); }

  void setRosStamp(const ros::Time & t)
  {
    messageRosStamp_ = t.toNSec();
    eventMsg_->header.stamp = t;
    triggerMsg_->header.stamp = t;
    waitForSensorTime_ = true;  // updated ros time, must wait for first time stamp
  }
  typename MsgType::Ptr & getEventMessage() { return (eventMsg_); }
  typename MsgType::Ptr & getTriggerMessage() { return (triggerMsg_); }

  bool hasEvents() const { return (!eventMsg_->events.empty()); }
  bool hasTriggers() const { return (!triggerMsg_->events.empty()); }

  void clearEvents()
  {
    maxEventSize_ = std::max(maxEventSize_, eventMsg_->events.size());
    eventMsg_->events.clear();
    eventMsg_->events.reserve(maxEventSize_);
  }

  void setBaseSensorTimeValid(bool b) { isBaseSensorTimeValid_ = b; }
  void setBaseSensorTime(uint64_t t)
  {
    baseSensorTime_ = t;
    setBaseSensorTimeValid(true);
  }

  void clearTriggers()
  {
    maxTriggerSize_ = std::max(maxTriggerSize_, triggerMsg_->events.size());
    triggerMsg_->events.clear();
    triggerMsg_->events.reserve(maxTriggerSize_);
  }

private:
  // ------------ variables
  uint64_t messageRosStamp_;
  uint64_t baseSensorTime_{0};
  bool isBaseSensorTimeValid_{false};
  bool waitForSensorTime_{true};
  typename MsgType::Ptr eventMsg_;
  typename MsgType::Ptr triggerMsg_;
  std::shared_ptr<Encoder> eventEncoder_;
  std::shared_ptr<Encoder> triggerEncoder_;
  size_t maxEventSize_{0};
  size_t maxTriggerSize_{0};
  bool isInitialized_{false};
  uint64_t eventSeq_{0};
  uint64_t triggerSeq_{0};
};

template <>
void MessageMaker<event_array_msgs::EventArray>::eventCD(
  uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity)
{
  if (!isBaseSensorTimeValid_) {
    setBaseSensorTime(sensor_time);
  }
  const uint32_t dt = static_cast<uint32_t>(sensor_time - baseSensorTime_);
  eventEncoder_->encodeCD(dt, ex, ey, polarity);
}

template <>
void MessageMaker<event_array_msgs::EventArray>::eventExtTrigger(
  uint64_t sensor_time, uint8_t edge, uint8_t id)
{
  if (!isBaseSensorTimeValid_) {
    setBaseSensorTime(sensor_time);
  }
  const uint32_t dt = static_cast<uint32_t>(sensor_time - baseSensorTime_);
  triggerEncoder_->encodeExtTrigger(dt, edge, id);
}

template <>
void MessageMaker<event_array_msgs::EventArray>::initializeMore()
{
  eventEncoder_->setBuffer(&(eventMsg_->events));
  triggerEncoder_->setBuffer(&(triggerMsg_->events));
  eventMsg_->is_bigendian = check_endian::isBigEndian();
  triggerMsg_->is_bigendian = check_endian::isBigEndian();
  eventMsg_->encoding = "mono";
  triggerMsg_->encoding = "trigger";
}

template <>
void MessageMaker<event_array_msgs::EventArray>::messageDone()
{
  if (eventEncoder_) {
    eventEncoder_->flush();
  }
  if (triggerEncoder_) {
    triggerEncoder_->flush();
  }
  eventMsg_->time_base = baseSensorTime_;
  triggerMsg_->time_base = baseSensorTime_;
  if (!eventMsg_->events.empty()) {
    eventMsg_->seq = eventSeq_;
    eventMsg_->header.seq = eventSeq_++;
  }
  if (!triggerMsg_->events.empty()) {
    triggerMsg_->seq = triggerSeq_;
    triggerMsg_->header.seq = triggerSeq_++;
  }
}

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

  inline void decode(const EventArray::ConstPtr & msg)
  {
    // decode message
    auto decIt = decoders_.find(msg->encoding);
    if (decIt == decoders_.end()) {
      auto dec = Decoder::newInstance(msg->encoding);
      if (dec) {
        decIt = decoders_.insert({msg->encoding, dec}).first;
      } else {
        printf("unsupported encoding: %s\n", msg->encoding.c_str());
        return;
      }
    }
    auto & decoder = *(decIt->second);
    decoder.decode(&msg->events[0], msg->events.size(), &messageMaker_);
  }

  void eventMsg(EventArray::ConstPtr msg)
  {
    if (!messageMaker_.isInitialized()) {
      // only called once!
      messageMaker_.initialize(msg->width, msg->height, msg->header);
    }
    messageMaker_.setRosStamp(msg->header.stamp);

    if (eventPub_.getNumSubscribers() || triggerPub_.getNumSubscribers()) {
      // decode message, produces callbacks into message maker
      decode(msg);
    }
    messageMaker_.messageDone();

    if (eventPub_.getNumSubscribers() && messageMaker_.hasEvents()) {
      eventPub_.publish(messageMaker_.getEventMessage());
    }
    if (triggerPub_.getNumSubscribers() && messageMaker_.hasTriggers()) {
      triggerPub_.publish(messageMaker_.getTriggerMessage());
    }
    messageMaker_.clearEvents();
    messageMaker_.clearTriggers();
    messageMaker_.setBaseSensorTimeValid(false);
  }
  // ---------- variables
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher eventPub_;
  ros::Publisher triggerPub_;

  std::unordered_map<std::string, std::shared_ptr<Decoder>> decoders_;
  MessageMaker<MsgType> messageMaker_;
};
}  // namespace event_array_tools
#endif  // EVENT_ARRAY_TOOLS__REPUBLISH_ROS1_H_
