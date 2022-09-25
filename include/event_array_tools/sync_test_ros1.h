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

#ifndef EVENT_ARRAY_TOOLS__SYNC_TEST_ROS1_H_
#define EVENT_ARRAY_TOOLS__SYNC_TEST_ROS1_H_

#include <event_array_codecs/decoder.h>
#include <event_array_codecs/event_processor.h>
#include <event_array_msgs/EventArray.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace event_array_tools
{
using EventArray = event_array_msgs::EventArray;
using EventArrayConstPtr = EventArray::ConstPtr;

class SyncTest;  // forward decl
struct EventSub
{
  void callback(EventArrayConstPtr events);
  // ----- variables --------
  ros::Subscriber sub_;
  std::unordered_map<std::string, std::shared_ptr<event_array_codecs::Decoder>> decoders_;
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
}  // namespace event_array_tools
#endif  // EVENT_ARRAY_TOOLS__SYNC_TEST_ROS1_H_
