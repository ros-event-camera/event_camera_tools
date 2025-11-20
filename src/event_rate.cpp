// -*-c++-*--------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>
#include <event_camera_tools/time_slicer.h>
#include <inttypes.h>
#include <unistd.h>

#include <chrono>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <unordered_map>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "event_rate [-b bag] [-r <rate_file>] [-t <trigger_file>] [-p period_ns] <ros_topic>"
            << std::endl;
}

using event_camera_codecs::Decoder;
using event_camera_codecs::EventPacket;

class EventRate
{
public:
  explicit EventRate(
    int64_t period, const std::string & rate_file, const std::string & trigger_file)
  : period_(period)
  {
    time_slicer_.setSensorPeriod(period_);
    time_slicer_.setProcessor(this);
    rate_file_.open(rate_file);
    if (!rate_file_.is_open()) {
      throw std::runtime_error("cannot open rate file!");
    }
    trigger_file_.open(trigger_file);
    if (!trigger_file_.is_open()) {
      throw std::runtime_error("cannot open trigger file!");
    }
  }

  ~EventRate()
  {
    if (rate_file_.is_open()) {
      rate_file_.close();
    }
    if (trigger_file_.is_open()) {
      trigger_file_.close();
    }
  }

  void eventCD(uint64_t /* sensor_time */, uint16_t /* ex */, uint16_t /* ey */, uint8_t polarity)
  {
    num_cd_events_[polarity]++;
  }
  bool eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t)
  {
    trigger_file_ << sensor_time << " " << static_cast<int>(edge) << std::endl;
    num_trigger_events_[edge]++;
    return (true);
  }

  void finished() {}

  void rawData(const char *, size_t) {}

  void timeSliceComplete(uint64_t sensor_time, const rclcpp::Time & ros_time)
  {
    rate_file_ << ros_time.nanoseconds() << " " << sensor_time << " " << num_cd_events_[0] << " "
               << num_cd_events_[1] << " " << num_trigger_events_[0] << " "
               << num_trigger_events_[1] << std::endl;
    num_cd_events_[0] = 0;
    num_cd_events_[1] = 0;
    num_trigger_events_[0] = 0;
    num_trigger_events_[1] = 0;
  }

  void eventMsg(EventPacket::ConstSharedPtr msg) { time_slicer_.eventMsg(msg); }

private:
  // ---------- variables
  event_camera_tools::TimeSlicer<EventRate> time_slicer_;
  size_t num_cd_events_[2]{0, 0};
  size_t num_trigger_events_[2]{0, 0};
  int64_t period_{0};
  std::ofstream rate_file_;
  std::ofstream trigger_file_;
};

class EventRateNode : public rclcpp::Node
{
public:
  explicit EventRateNode(
    const rclcpp::NodeOptions & options, const std::string & topic, EventRate * event_rate)
  : Node("event_rate", options), event_rate_(event_rate)
  {
    const int qsize = 1000;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qsize)).best_effort().durability_volatile();
    RCLCPP_INFO_STREAM(this->get_logger(), "subscribing to " << topic);
    sub_ = this->create_subscription<EventPacket>(
      topic, qos, std::bind(&EventRateNode::eventMsg, this, std::placeholders::_1));
  }
  void eventMsg(EventPacket::ConstSharedPtr msg) { event_rate_->eventMsg(msg); }

private:
  // ---------- variables
  rclcpp::Subscription<EventPacket>::SharedPtr sub_;
  EventRate * event_rate_;
};

static void read_from_bag(
  const std::string & bagName, const std::string & topic, EventRate * event_rate)
{
  rosbag2_cpp::Reader reader;
  reader.open(bagName);
  rclcpp::Serialization<EventPacket> serialization;
  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (msg->topic_name == topic) {
      rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
      auto m = std::make_shared<EventPacket>();
      serialization.deserialize_message(&serializedMsg, m.get());
      event_rate->eventMsg(m);
    }
  }
}

int main(int argc, char ** argv)
{
  int opt;

  std::string topic;
  std::string bagFile;
  int64_t period = 1000000LL;  // 1ms
  std::string trigger_file = "triggers.txt";
  std::string rate_file = "event_rate.txt";
  rclcpp::init(argc, argv);
  while ((opt = getopt(argc, argv, "b:t:r:p:")) != -1) {
    switch (opt) {
      case 'b':
        bagFile = optarg;
        break;
      case 'h':
        usage();
        return (-1);
        break;
      case 't':
        trigger_file = optarg;
        break;
      case 'r':
        rate_file = optarg;
        break;
      case 'p':
        period = std::atoll(optarg);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (optind != argc - 1) {
    std::cout << "expected topic!" << std::endl;
    usage();
    return (-1);
  }
  topic = argv[optind];
  EventRate event_rate(period, rate_file, trigger_file);
  if (bagFile.empty()) {
    auto node = std::make_shared<EventRateNode>(rclcpp::NodeOptions(), topic, &event_rate);
    rclcpp::spin(node);  // should not return
    rclcpp::shutdown();
  } else {
    read_from_bag(bagFile, topic, &event_rate);
  }
  return 0;
}
