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

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>
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
  std::cout << "echo [-b bag] [-n (no headers)] [-t (triggers only)] <ros_topic>" << std::endl;
}

using event_camera_codecs::Decoder;
using event_camera_codecs::EventPacket;

class Echo : public event_camera_codecs::EventProcessor
{
public:
  explicit Echo(bool printHeaders, bool triggersOnly)
  : printHeaders_(printHeaders), triggersOnly_(triggersOnly)
  {
  }
  // ---------- from the EventProcessor interface:
  void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    if (!triggersOnly_) {
      printf("%8" PRIu64 " %4d %4d %1d\n", sensor_time, ex, ey, polarity);
    }
    numCDEvents_[polarity]++;
    if (cdStamps_[0] == 0) {
      cdStamps_[0] = sensor_time;
    }
    cdStamps_[1] = std::max(cdStamps_[1], sensor_time);
  }
  bool eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t id) override
  {
    printf("%8" PRIu64 " edge: %1d  id: %2d\n", sensor_time, edge, id);
    numTrigEvents_[edge]++;
    if (trigStamps_[0] == 0) {
      trigStamps_[0] = sensor_time;
    }
    trigStamps_[1] = std::max(trigStamps_[1], sensor_time);
    return (true);
  }
  void finished() override {}
  void rawData(const char *, size_t) override {}
  // -------- end of inherited

  void eventMsg(EventPacket::ConstSharedPtr msg)
  {
    if (printHeaders_) {
      printf("-------------------------------\n");
      printf("width: %4d  height: %4d enc: %s\n", msg->width, msg->height, msg->encoding.c_str());
      printf("header stamp: %8" PRIu64 "\n", rclcpp::Time(msg->header.stamp).nanoseconds());
      printf("time base: %8" PRIu64 "\n", msg->time_base);
      printf("seqno: %8" PRIu64 "\n", msg->seq);
      printf("---\n");
    }

    auto decoder = decoderFactory_.getInstance(msg->encoding, msg->width, msg->height);
    if (!decoder) {
      printf("unsupported encoding: %s\n", msg->encoding.c_str());
      return;
    }
    decoder->setTimeBase(msg->time_base);
    decoder->decode(&msg->events[0], msg->events.size(), this);
  }
  const size_t * getNumCDEvents() { return (numCDEvents_); }
  const size_t * getNumTrigEvents() { return (numTrigEvents_); }
  const uint64_t * getCDStamps() { return (cdStamps_); }
  const uint64_t * getTrigStamps() { return (trigStamps_); }

private:
  // ---------- variables
  event_camera_codecs::DecoderFactory<EventPacket, Echo> decoderFactory_;
  size_t numCDEvents_[2]{0, 0};
  size_t numTrigEvents_[2]{0, 0};
  uint64_t cdStamps_[2]{0, 0};
  uint64_t trigStamps_[2]{0, 0};
  bool printHeaders_{false};
  bool triggersOnly_{false};
};

class EchoNode : public rclcpp::Node
{
public:
  explicit EchoNode(const rclcpp::NodeOptions & options, const std::string & topic, Echo * echo)
  : Node("echo", options), echo_(echo)
  {
    const int qsize = 1000;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qsize)).best_effort().durability_volatile();
    RCLCPP_INFO_STREAM(this->get_logger(), "subscribing to " << topic);
    sub_ = this->create_subscription<EventPacket>(
      topic, qos, std::bind(&EchoNode::eventMsg, this, std::placeholders::_1));
  }
  void eventMsg(EventPacket::ConstSharedPtr msg) { echo_->eventMsg(msg); }

private:
  // ---------- variables
  rclcpp::Subscription<EventPacket>::SharedPtr sub_;
  Echo * echo_;
};

static void read_from_bag(const std::string & bagName, const std::string & topic, Echo * echo)
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
      echo->eventMsg(m);
    }
  }
}

int main(int argc, char ** argv)
{
  int opt;

  std::string topic;
  std::string bagFile;
  bool printHeaders(true);
  bool triggersOnly(false);
  rclcpp::init(argc, argv);
  while ((opt = getopt(argc, argv, "b:nt")) != -1) {
    switch (opt) {
      case 'b':
        bagFile = optarg;
        break;
      case 'h':
        usage();
        return (-1);
        break;
      case 'n':
        printHeaders = false;
        break;
      case 't':
        triggersOnly = true;
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
  Echo echo(printHeaders, triggersOnly);
  if (bagFile.empty()) {
    auto node = std::make_shared<EchoNode>(rclcpp::NodeOptions(), topic, &echo);
    rclcpp::spin(node);  // should not return
    rclcpp::shutdown();
  } else {
    read_from_bag(bagFile, topic, &echo);
    size_t totCDEvents = echo.getNumCDEvents()[0] + echo.getNumCDEvents()[1];
    size_t totTrigEvents = echo.getNumTrigEvents()[0] + echo.getNumTrigEvents()[1];
    std::cout << "cd      ev: " << totCDEvents << " time: " << echo.getCDStamps()[0] << " -> "
              << echo.getCDStamps()[1] << std::endl;
    std::cout << "trigger ev: " << totTrigEvents << " time: " << echo.getTrigStamps()[0] << " -> "
              << echo.getTrigStamps()[1] << std::endl;
  }
  return 0;
}
