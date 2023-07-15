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
#include <unistd.h>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include "event_camera_tools/movie_maker.h"

using event_camera_codecs::EventPacket;

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "movie_maker -b name_of_bag_file -t topic -f fps" << std::endl;
}

static size_t process_bag(const std::string & inFile, const std::string & topic, const double fps)
{
  size_t numBytes(0);
  event_camera_codecs::DecoderFactory<EventPacket, event_camera_tools::MovieMaker> decoderFactory;
  rosbag2_cpp::Reader reader;
  reader.open(inFile);
  rclcpp::Serialization<EventPacket> serialization;
  event_camera_tools::MovieMaker maker;
  maker.setFramePeriod(1.0 / fps);
  rclcpp::Time t0;
  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (msg->topic_name == topic) {
      rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
      EventPacket m;
      serialization.deserialize_message(&serializedMsg, &m);
      if (numBytes == 0) {
        maker.resetImage(m.width, m.height);
        t0 = m.header.stamp;
      }
      auto decoder = decoderFactory.getInstance(m.encoding, m.width, m.height);
      if (!decoder) {
        std::cout << "unknown encoding: " << m.encoding << std::endl;
        continue;
      }
      decoder->setTimeBase(m.time_base);
      decoder->decode(&(m.events[0]), m.events.size(), &maker);
      numBytes += m.events.size();
    }
  }
  return (numBytes);
}

int main(int argc, char ** argv)
{
  int opt;

  std::string bagFile;
  std::string topic("/event_camera/events");
  double fps(25);
  while ((opt = getopt(argc, argv, "b:t:f:h")) != -1) {
    switch (opt) {
      case 'b':
        bagFile = optarg;
        break;
      case 't':
        topic = optarg;
        break;
      case 'f':
        fps = atof(optarg);
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

  std::filesystem::create_directories(std::string("movie_frames"));

  auto start = std::chrono::high_resolution_clock::now();
  const size_t numBytes = process_bag(bagFile, topic, fps);
  auto final = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  std::cout << "number of bytes read: " << numBytes * 1e-6 << " MB in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "bytes rate: " << static_cast<double>(numBytes) / total_duration.count() << " MB/s"
            << std::endl;
  return (0);
}
