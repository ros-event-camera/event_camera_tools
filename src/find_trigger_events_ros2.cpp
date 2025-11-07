// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#include <unistd.h>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

using event_camera_msgs::msg::EventPacket;

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "find_trigger_events -i input_bag [-t trigger_topic] " << std::endl;
}

class TriggerFinder : public event_camera_codecs::EventProcessor
{
public:
  void eventCD(uint64_t t, uint16_t, uint16_t, uint8_t) override
  {
    if (newMessageStarted_) {
      sensorMsgTime_ = t;
      newMessageStarted_ = false;
    }
  }
  bool eventExtTrigger(uint64_t sensor_time, uint8_t, uint8_t) override
  {
    triggerTimes_.push_back({rosMsgTime_ + (sensor_time - sensorMsgTime_), sensor_time});
    return (true);
  }
  void finished() override {}
  void rawData(const char *, size_t) override {}

  const auto & getTriggerTimes() const { return (triggerTimes_); }
  void setRosTime(uint64_t t)
  {
    newMessageStarted_ = true;
    rosMsgTime_ = t;
  }

private:
  std::vector<std::pair<uint64_t, uint64_t>> triggerTimes_;
  uint64_t rosMsgTime_{0};
  uint64_t sensorMsgTime_{0};
  bool newMessageStarted_{false};
};

int main(int argc, char ** argv)
{
  int opt;
  std::string inBagName;
  std::string topic;
  while ((opt = getopt(argc, argv, "i:t:")) != -1) {
    switch (opt) {
      case 'i':
        inBagName = optarg;
        break;
      case 't':
        topic = optarg;
        break;
      case 'h':
        usage();
        return (-1);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }

  if (inBagName.empty()) {
    std::cout << "missing input bag file name!" << std::endl;
    usage();
    return (-1);
  }

  if (topic.empty()) {
    std::cout << "no input topic found!" << std::endl;
    return (-1);
  }
  rosbag2_cpp::Reader reader;
  reader.open(inBagName);
  rclcpp::Serialization<EventPacket> serialization;
  size_t numMessages(0);
  TriggerFinder trigFind;
  event_camera_codecs::DecoderFactory<EventPacket, TriggerFinder> decoderFactory;

  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (msg->topic_name == topic) {
      rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
      std::shared_ptr<EventPacket> m(new EventPacket());
      serialization.deserialize_message(&serializedMsg, m.get());
      const auto decoder = decoderFactory.getInstance(*m);
      trigFind.setRosTime(rclcpp::Time(m->header.stamp).nanoseconds());
      decoder->decode(*m, &trigFind);
      numMessages++;
    }
  }
  const auto & tt = trigFind.getTriggerTimes();
  if (tt.empty()) {
    std::cout << "BUMMER: no trigger times found" << std::endl;
    return (0);
  }

  std::cout << "first trigger ROS    time: " << tt[0].first << std::endl;
  std::cout << "first trigger sensor time: " << tt[0].second << std::endl;

  uint64_t diff_ros{0};
  uint64_t diff_sensor{0};
  for (size_t i = 1; i < tt.size(); i++) {
    diff_ros += tt[i].first - tt[i - 1].first;
    diff_sensor += tt[i].second - tt[i - 1].second;
  }

  std::cout << "num triggers: " << tt.size() << std::endl;
  std::cout << "avg time between triggers:" << std::endl;
  std::cout << "ROS time:    " << (diff_ros / tt.size()) * 1e-9 << "s" << std::endl;
  std::cout << "sensor time: " << (diff_sensor / tt.size()) * 1e-9 << "s" << std::endl;
  std::cout << "processed " << numMessages << " number of messages" << std::endl;

  return (0);
}
