// -*-c++-*----------------------------------------------------------------------------------------
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

#include "event_camera_tools/sync_test_ros1.h"

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "sync_test <event_topic_cam_0> <event_topic_cam_1>" << std::endl;
}

namespace event_camera_tools
{
void EventSub::callback(EventPacketConstPtr msg)
{
  lastHeaderStamp_ = ros::Time(msg->header.stamp).toNSec();
  if (!msg->events.empty()) {
    auto decoder = decoderFactory_.getInstance(msg->encoding, msg->width, msg->height);
    if (!decoder) {
      printf("unsupported encoding: %s\n", msg->encoding.c_str());
      return;
    }
    decoder->setTimeBase(msg->time_base);
    uint64_t t_sensor;
    if (decoder->findFirstSensorTime(&msg->events[0], msg->events.size(), &t_sensor)) {
      syncTest_->updateStats(id_, ros::Time(msg->header.stamp).toNSec(), msg->time_base, t_sensor);
      lastSensorTime_ = t_sensor;
    }
  }
}

SyncTest::SyncTest(
  const std::string & topic_0, const std::string & topic_1, const ros::NodeHandle & nh)
: nh_(nh)
{
  resetVariables();
  eventSub_[0].syncTest_ = this;  // give back pointer
  eventSub_[0].id_ = 0;
  eventSub_[1].syncTest_ = this;  // give back pointer
  eventSub_[1].id_ = 1;

  timer_ = nh_.createTimer(ros::Duration(1.0), &SyncTest::timerExpired, this);
  eventSub_[0].sub_ = nh_.subscribe(topic_0, 1, &EventSub::callback, &eventSub_[0]);
  eventSub_[1].sub_ = nh_.subscribe(topic_1, 1, &EventSub::callback, &eventSub_[1]);
}

void SyncTest::updateStats(uint8_t id, uint64_t t_stamp, uint64_t t_base, uint64_t t_sensor)
{
  (void)t_stamp;
  (void)t_base;
  count_[id]++;
  int64_t dt = (id == 0) ? (t_sensor - eventSub_[1].lastSensorTime_)
                         : (-(t_sensor - eventSub_[0].lastSensorTime_));
  maxDiff_ = std::max(maxDiff_, dt);
  minDiff_ = std::min(minDiff_, dt);
  sumOfDiffs_ += dt * 1e-9;
}

void SyncTest::timerExpired(const ros::TimerEvent &)
{
  if (count_[0] && count_[1] > 0) {
    const auto count = count_[0] + count_[1];
    const double avg = sumOfDiffs_ / count;
    printf("avg sensor diff: %8.5lfs, count: %5zu\n", avg, count);
  } else {
    ROS_WARN_STREAM("no messages received: cam0: " << count_[0] << " cam1:  " << count_[1]);
  }
  resetVariables();
}

void SyncTest::resetVariables()
{
  minDiff_ = std::numeric_limits<int64_t>::max();
  maxDiff_ = std::numeric_limits<int64_t>::min();
  count_[0] = count_[1] = 0;
  sumOfDiffs_ = 0;
}
}  // namespace event_camera_tools

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "echo");
  ros::NodeHandle pnh("~");

  std::string topic_0;
  std::string topic_1;
  switch (argc) {
    case 3:
      topic_0 = argv[1];
      topic_1 = argv[2];
      break;
    default:
      usage();
      exit(-1);
  }
  try {
    event_camera_tools::SyncTest node(topic_0, topic_1, pnh);
    ros::spin();
  } catch (const std::exception & e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
    return (-1);
  }
  return (0);
}
