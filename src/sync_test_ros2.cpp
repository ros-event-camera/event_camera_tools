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

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "sync_test <event_topic_cam_0> <event_topic_cam_1>" << std::endl;
}

namespace event_camera_tools
{
using EventPacket = event_camera_msgs::msg::EventPacket;
using EventPacketConstPtr = EventPacket::ConstSharedPtr;

class SyncTest;  // forward decl
struct EventSub
{
  void callback(EventPacketConstPtr msg);
  // ----- variables --------
  rclcpp::Subscription<EventPacket>::SharedPtr sub_;
  event_camera_codecs::DecoderFactory<EventPacket> decoderFactory_;
  uint64_t lastHeaderStamp_{0};
  uint64_t lastSensorTime_{0};
  SyncTest * syncTest_{0};
  uint8_t id_{0};
};

class SyncTest : public rclcpp::Node
{
public:
  explicit SyncTest(
    const std::string & topic_0, const std::string & topic_1, const rclcpp::NodeOptions & options)
  : Node("sync_test", options)
  {
    resetVariables();
    eventSub_[0].syncTest_ = this;  // give back pointer
    eventSub_[0].id_ = 0;
    eventSub_[1].syncTest_ = this;  // give back pointer
    eventSub_[1].id_ = 1;

    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(1.0),
      [=]() { this->timerExpired(); });

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    eventSub_[0].sub_ = this->create_subscription<EventPacket>(
      topic_0, qos, std::bind(&EventSub::callback, &eventSub_[0], std::placeholders::_1));
    eventSub_[1].sub_ = this->create_subscription<EventPacket>(
      topic_1, qos, std::bind(&EventSub::callback, &eventSub_[1], std::placeholders::_1));
  }

  void updateStats(uint8_t id, uint64_t t_stamp, uint64_t t_base, uint64_t t_sensor)
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

private:
  void timerExpired()
  {
    if (count_[0] && count_[1] > 0) {
      const auto count = count_[0] + count_[1];
      const double avg = sumOfDiffs_ / count;
      printf("avg sensor diff: %8.5lfs, count: %5zu\n", avg, count);
    } else {
      RCLCPP_WARN_STREAM(
        get_logger(), "no messages received: cam0: " << count_[0] << " cam1:  " << count_[1]);
    }
    resetVariables();
  }

  void resetVariables()
  {
    minDiff_ = std::numeric_limits<int64_t>::max();
    maxDiff_ = std::numeric_limits<int64_t>::min();
    count_[0] = count_[1] = 0;
    sumOfDiffs_ = 0;
  }
  // ---------  variables
  rclcpp::TimerBase::SharedPtr timer_;
  EventSub eventSub_[2];
  double sumOfDiffs_{0};
  uint64_t count_[2]{0, 0};
  int64_t maxDiff_;
  int64_t minDiff_;
};

// must define here because of reference to SyncTest class
void EventSub::callback(EventPacketConstPtr msg)
{
  lastHeaderStamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (!msg->events.empty()) {
    auto decoder = decoderFactory_.getInstance(msg->encoding, msg->width, msg->height);
    if (!decoder) {
      printf("unsupported encoding: %s\n", msg->encoding.c_str());
      return;
    }
    decoder->setTimeBase(msg->time_base);
    uint64_t t_sensor;
    if (decoder->findFirstSensorTime(&msg->events[0], msg->events.size(), &t_sensor)) {
      syncTest_->updateStats(
        id_, rclcpp::Time(msg->header.stamp).nanoseconds(), msg->time_base, t_sensor);
      lastSensorTime_ = t_sensor;
    }
  }
}

}  // namespace event_camera_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

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
  auto node =
    std::make_shared<event_camera_tools::SyncTest>(topic_0, topic_1, rclcpp::NodeOptions());
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return (0);
}
