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

#include <event_array_msgs/decode.h>
#include <unistd.h>

#include <chrono>
#include <event_array_msgs/msg/event_array.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "perf <ros_topic>" << std::endl;
}

using event_array_msgs::msg::EventArray;
class Perf : public rclcpp::Node
{
public:
  explicit Perf(const std::string & topic, const rclcpp::NodeOptions & options)
  : Node("perf", options)
  {
    const int qsize = 1000;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qsize)).best_effort().durability_volatile();
    RCLCPP_INFO_STREAM(this->get_logger(), "subscribing to " << topic);
    sub_ = this->create_subscription<EventArray>(
      topic, qos, std::bind(&Perf::eventMsg, this, std::placeholders::_1));
    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(2.0),
      [=]() { this->timerExpired(); });
    lastTime_ = this->get_clock()->now();
  }
  void eventMsg(EventArray::ConstSharedPtr msg)
  {
    size_t idx = 0;
    size_t bytes_per_event = 0;
    if (msg->encoding == "mono") {
      idx = 0;
      bytes_per_event = event_array_msgs::mono::bytes_per_event;
    } else if (msg->encoding == "trigger") {
      idx = 1;
      bytes_per_event = event_array_msgs::trigger::bytes_per_event;
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "unsupported encoding: " << msg->encoding);
      throw std::runtime_error("unsupported encoding");
    }
    const rclcpp::Time t_stamp = rclcpp::Time(msg->header.stamp);
    if (lastSeq_[idx] == 0) {
      lastSeq_[idx] = msg->seq - 1;
      lastStamp_ = t_stamp;
    }
    numMsgs_[idx]++;
    const int num_dropped = msg->seq - 1 - lastSeq_[idx];
    dropped_[idx] += num_dropped;
    lastSeq_[idx] = msg->seq;
    if (t_stamp < lastStamp_) {
      numReverse_[idx]++;
    }
    lastStamp_ = t_stamp;
    numEvents_[idx] += msg->events.size() / bytes_per_event;
    const auto t = this->get_clock()->now();
    delay_[idx] += (t - rclcpp::Time(msg->header.stamp)).nanoseconds();
  }

  void timerExpired()
  {
    const auto t = this->get_clock()->now();
    const double dt = (t - lastTime_).nanoseconds() * 1e-9;  // in milliseconds
    for (int i = 0; i < 2; i++) {
      if (numEvents_[i] != 0 && numMsgs_[i] != 0) {
        const std::string label = i == 0 ? "events" : "triggr";
        printf(
          "%s: %8.4f M/s msgs: %8.2f/s drop: %3zu delay: %5.2fms", label.c_str(),
          numEvents_[i] / dt * 1e-6, numMsgs_[i] / dt, dropped_[i],
          delay_[i] / (1e6 * numMsgs_[i]));
        if (numReverse_[i] != 0) {
          printf(" ROS STAMP REVERSED: %3zu\n", numReverse_[i]);
        } else {
          printf("\n");
        }
      }
      numEvents_[i] = numMsgs_[i] = delay_[i] = dropped_[i] = numReverse_[i] = 0;
    }
    lastTime_ = t;
  }
  // ---------- variables
  rclcpp::Subscription<EventArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t numMsgs_[2]{0, 0};
  size_t numEvents_[2]{0, 0};
  int64_t delay_[2]{0, 0};
  size_t dropped_[2]{0, 0};
  size_t numReverse_[2]{0, 0};
  uint64_t lastSeq_[2]{0, 0};
  rclcpp::Time lastTime_;
  rclcpp::Time lastStamp_;
};

int main(int argc, char ** argv)
{
  std::string topic;
  rclcpp::init(argc, argv);
  switch (argc) {
    case 2:
      topic = argv[1];
      break;
    default:
      usage();
      exit(-1);
  }
  auto node = std::make_shared<Perf>(topic, rclcpp::NodeOptions());
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return 0;
}
