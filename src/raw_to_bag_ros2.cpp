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
#include <event_array_msgs/encode.h>
#include <unistd.h>

#include <chrono>
#include <event_array_msgs/msg/event_array.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "evt_3_utils.h"

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "raw_to_bag -b name_of_bag_file -i name_of_raw_file -t topic -f "
               "frame_id -w width -h height -e encoding"
            << std::endl;
}

namespace event_array_tools
{
using event_array_msgs::mono::bytes_per_event;
using event_array_msgs::msg::EventArray;

static bool is_big_endian()
{
  const union {
    uint32_t i;
    char c[4];
  } combined_int = {0x01020304};  // from stackoverflow
  return (combined_int.c[0] == 1);
}

class MessageUpdaterMono : public evt_3_utils::MessageUpdater
{
public:
  explicit MessageUpdaterMono(
    const std::string & bagName, const std::string & topic, const std::string & frameId,
    uint32_t width, uint32_t height)
  : t0_ros_(rclcpp::Clock().now()), topic_(topic)
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(bagName);
    writer_->create_topic(
      {topic, "event_array_msgs/msg/EventArray", rmw_get_serialization_format(), ""});
    topic_ = topic;
    msg_.header.frame_id = frameId;
    msg_.width = width;
    msg_.height = height;
    msg_.encoding = "mono";
    msg_.is_bigendian = is_big_endian();
    msg_.seq = 0;
  }
  ~MessageUpdaterMono() { writer_.reset(); }

  void addEvent(uint64_t ts_ros, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    if (msg_.events.empty()) {  // starting new message
      msg_.header.stamp = rclcpp::Time(ts_ros, RCL_SYSTEM_TIME);
      msg_.time_base = ts_ros;
    }
    // nanoseconds since start of message
    uint32_t dt = ts_ros - msg_.time_base;  // should not overflow
    constexpr size_t kBufSize = bytes_per_event;
    uint8_t buffer[kBufSize];
    event_array_msgs::mono::encode(
      reinterpret_cast<uint64_t *>(buffer), static_cast<bool>(polarity), ex, ey, dt);
    msg_.events.insert(msg_.events.end(), buffer, buffer + bytes_per_event);
    const size_t MAX_MESSAGE_SIZE(20000);
    // keep MAX_MESSAGE_TIME small enough to not overflow the uint32_t!
    const uint32_t MAX_MESSAGE_TIME(1000000000);  // in nanoseconds = 1ms
    if (msg_.events.size() >= MAX_MESSAGE_SIZE || dt > MAX_MESSAGE_TIME) {
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<EventArray> serialization;
      serialization.serialize_message(&msg_, &serialized_msg);
      writer_->write(
        serialized_msg, topic_, "event_array_msgs/msg/EventArray", rclcpp::Time(msg_.header.stamp));
      msg_.events.clear();
      msg_.seq++;
    }
    numEvents_++;
  }

  uint64_t getROSTime() override { return (t0_ros_.nanoseconds()); }
  void bufferRead(const char *, size_t) override {}

  void finished() override
  {
    std::cout << "wrote " << numEvents_ << " events to bag in " << msg_.seq
              << " messages, msg/event: " << numEvents_ / msg_.seq << std::endl;
  }
  // ---------- variables
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  EventArray msg_;
  rclcpp::Time t0_ros_;
  std::string topic_;
  size_t numEvents_{0};
};
class MessageUpdaterEvt3 : public evt_3_utils::MessageUpdater
{
public:
  explicit MessageUpdaterEvt3(
    const std::string & bagName, const std::string & topic, const std::string & frameId,
    uint32_t width, uint32_t height)
  : t0_ros_(rclcpp::Clock().now()), topic_(topic)
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(bagName);
    writer_->create_topic(
      {topic, "event_array_msgs/msg/EventArray", rmw_get_serialization_format(), ""});
    topic_ = topic;
    msg_.header.frame_id = frameId;
    msg_.width = width;
    msg_.height = height;
    msg_.encoding = "evt3";
    msg_.is_bigendian = is_big_endian();
    msg_.seq = 0;
  }

  ~MessageUpdaterEvt3() { writer_.reset(); }

  void addEvent(uint64_t ts_ros, uint16_t, uint16_t, uint8_t) override
  {
    if (msg_.events.empty()) {  // starting new message
      msg_.header.stamp = rclcpp::Time(ts_ros, RCL_SYSTEM_TIME);
      msg_.time_base = ts_ros;
      msg_.events.push_back(0);  // to mark message as started
    }
    numEvents_++;
  }

  uint64_t getROSTime() override { return (t0_ros_.nanoseconds()); }
  void bufferRead(const char * data, size_t len) override
  {
    msg_.events.clear();  // remove marker
    msg_.events.insert(msg_.events.end(), data, data + len);
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<EventArray> serialization;
    serialization.serialize_message(&msg_, &serialized_msg);
    writer_->write(
      serialized_msg, topic_, "event_array_msgs/msg/EventArray", rclcpp::Time(msg_.header.stamp));
    msg_.events.clear();  // mark as new
    msg_.seq++;
  }

  void finished() override
  {
    std::cout << "wrote " << numEvents_ << " events to bag in " << msg_.seq
              << " messages, msg/event: " << numEvents_ / msg_.seq << std::endl;
  }
  // ---------- variables
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  EventArray msg_;
  rclcpp::Time t0_ros_;
  std::string topic_;
  size_t numEvents_{0};
};

}  // namespace event_array_tools

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outFile;
  std::string topic("/event_camera/events");
  std::string frameId("event_camera");
  std::string encoding("mono");
  int height(480);
  int width(640);
  while ((opt = getopt(argc, argv, "b:i:t:f:h:w:e:")) != -1) {
    switch (opt) {
      case 'b':
        outFile = optarg;
        break;
      case 'i':
        inFile = optarg;
        break;
      case 't':
        topic = optarg;
        break;
      case 'f':
        frameId = optarg;
        break;
      case 'w':
        width = atoi(optarg);
        break;
      case 'h':
        height = atoi(optarg);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (inFile.empty() || outFile.empty()) {
    std::cout << "missing input or output file name!" << std::endl;
    usage();
    return (-1);
  }
  auto start = std::chrono::high_resolution_clock::now();

  size_t numEvents = 0;
  decltype(start) final;
  if (encoding == "mono") {
    event_array_tools::MessageUpdaterMono updater(outFile, topic, frameId, width, height);
    numEvents = event_array_tools::evt_3_utils::read(inFile, &updater);
    final = std::chrono::high_resolution_clock::now();
  } else if (encoding == "evt3") {
    event_array_tools::MessageUpdaterEvt3 updater(outFile, topic, frameId, width, height);
    numEvents = event_array_tools::evt_3_utils::read(inFile, &updater);
    final = std::chrono::high_resolution_clock::now();
  } else {
    std::cout << "invalid encoding: " << encoding << std::endl;
  }
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  std::cout << "number of events read: " << numEvents * 1e-6 << " Mev in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "event rate: " << static_cast<double>(numEvents) / total_duration.count() << " Mevs"
            << std::endl;
  return (0);
}
