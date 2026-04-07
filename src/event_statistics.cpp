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

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_tools/event_statistics.h>
#include <unistd.h>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

using event_camera_codecs::EventPacket;

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "event_statistics -b name_of_bag_file -t topic [-s scale_file] [-m max_delta_time]"
            << std::endl;
}

static std::tuple<size_t, size_t, int64_t, int64_t> process_bag(
  const std::string & inFile, const std::string & topic, const std::string & scale_file,
  double max_delta_time)
{
  size_t numBytes(0);
  event_camera_codecs::DecoderFactory<EventPacket, event_camera_tools::EventStatistics>
    decoderFactory;
  rosbag2_cpp::Reader reader;
  reader.open(inFile);
  rclcpp::Serialization<EventPacket> serialization;
  event_camera_tools::EventStatistics stats(scale_file);
  stats.setMaxDeltaTime(static_cast<uint64_t>(max_delta_time * 1e9));
  rclcpp::Time t0;
  bool topic_found = false;
  int64_t start_time = 0;
  int64_t end_time = 0;
  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (msg->topic_name == topic) {
      rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
      EventPacket m;
      serialization.deserialize_message(&serializedMsg, &m);
      end_time = rclcpp::Time(m.header.stamp).nanoseconds();
      if (!topic_found) {
        stats.initialize(m.width, m.height);
        start_time = end_time;
      }
      auto decoder = decoderFactory.getInstance(m.encoding, m.width, m.height);
      if (!decoder) {
        std::cout << "unknown encoding: " << m.encoding << std::endl;
        continue;
      }
      decoder->setTimeBase(m.time_base);
      decoder->decode(&(m.events[0]), m.events.size(), &stats);
      numBytes += m.events.size();
      topic_found = true;
    }
  }
  if (!topic_found) {
    throw std::runtime_error("topic not found!");
  }
  return {numBytes, stats.getNumEvents(), stats.getTimePassed(), end_time - start_time};
}

int main(int argc, char ** argv)
{
  int opt;

  std::string bagFile;
  std::string topic("/event_camera/events");
  std::string scale_file("scale.txt");
  double max_delta_time{5e-3};
  while ((opt = getopt(argc, argv, "b:t:m:s:h")) != -1) {
    switch (opt) {
      case 'b':
        bagFile = optarg;
        break;
      case 't':
        topic = optarg;
        break;
      case 's':
        scale_file = optarg;
        break;
      case 'm':
        max_delta_time = std::stod(optarg);
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
  if (bagFile.empty()) {
    std::cout << "missing bag file name!" << std::endl;
    usage();
    return (-1);
  }
  std::cout << "using topic: " << topic << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  const auto [numBytes, numEvents, duration_sensor, duration_host] =
    process_bag(bagFile, topic, scale_file, max_delta_time);
  auto final = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);
  std::cout << "sensor time elapsed: " << duration_sensor * 1e-9 << std::endl;
  std::cout << "host   time elapsed: " << duration_host * 1e-9 << std::endl;
  std::cout << "host-sensor time difference: " << (duration_host - duration_sensor) * 1e-9
            << std::endl;
  std::cout << "number of bytes read: " << numBytes * 1e-6 << " MB in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "bytes rate: " << static_cast<double>(numBytes) / total_duration.count() << " MB/s"
            << std::endl;
  std::cout << "event rate: " << static_cast<double>(numEvents) / total_duration.count() << " Mev/s"
            << std::endl;
  std::cout << "output written to " << scale_file << std::endl;
  return (0);
}
