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

#ifndef EVENT_CAMERA_TOOLS__SYNC_TEST_ROS1_H_
#define EVENT_CAMERA_TOOLS__SYNC_TEST_ROS1_H_

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>
#include <event_camera_msgs/EventPacket.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace event_camera_tools
{
using EventPacket = event_camera_codecs::EventPacket;
using EventPacketConstPtr = EventPacket::ConstPtr;

class SyncTest;  // forward decl
struct EventSub
{
  void callback(EventPacketConstPtr events);
  // ----- variables --------
  ros::Subscriber sub_;
  event_camera_codecs::DecoderFactory<EventPacket> decoderFactory_;
  uint64_t lastHeaderStamp_{0};
  uint64_t lastSensorTime_{0};
  SyncTest * syncTest_{0};
  uint8_t id_{0};
};

class SyncTest
{
public:
  explicit SyncTest(
    const std::string & topic_0, const std::string & topic_1, const ros::NodeHandle & nh);
  void updateStats(uint8_t id, uint64_t t_stamp, uint64_t t_base, uint64_t t_sensor);

private:
  bool start();
  void resetVariables();
  void timerExpired(const ros::TimerEvent &);

  // ---------  variables
  ros::NodeHandle nh_;
  ros::Timer timer_;
  EventSub eventSub_[2];
  double sumOfDiffs_{0};
  uint64_t count_[2]{0, 0};
  int64_t maxDiff_;
  int64_t minDiff_;
};
}  // namespace event_camera_tools
#endif  // EVENT_CAMERA_TOOLS__SYNC_TEST_ROS1_H_
