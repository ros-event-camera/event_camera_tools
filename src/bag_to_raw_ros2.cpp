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

#include <unistd.h>

#include <chrono>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_raw -b name_of_bag_file -o name_of_raw_file -t topic -c camera" << std::endl;
}

namespace event_camera_tools
{
using event_camera_msgs::msg::EventPacket;

std::map<std::string, std::string> headers = {
  {"silkyev",
   "% date 2022-07-20 11:20:09\n"
   "% evt 3.0\n"
   "% firmware_version 4.1.1\n"
   "% integrator_name CenturyArks\n"
   "% plugin_name evc3a_plugin_gen31\n"
   "% serial_number 00000198\n"
   "% subsystem_ID 0\n"
   "% system_ID 40\n"},

  {"evk4",
   "% date 2022-09-14 17:11:11\n"
   "% evt 3.0\n"
   "% firmware_version 0.0.0\n"
   "% format EVT3\n"
   "% geometry 1280x720\n"
   "% integrator_name Prophesee\n"
   "% plugin_name hal_plugin_imx636_evk4\n"
   "% sensor_generation 4.2\n"
   "% serial_number 00050108\n"
   "% system_ID 49\n"}};

size_t write(
  std::fstream & out, const uint8_t * p, const size_t num_bytes, const uint64_t time_base,
  const std::string & encoding, uint32_t * last_evt_stamp)
{
  (void)time_base;
  (void)last_evt_stamp;
  if (encoding == "evt3") {
    out.write(reinterpret_cast<const char *>(p), num_bytes);
  } else {
    std::cout << "only evt3 is supported!" << std::endl;
    throw std::runtime_error("only evt3 supported!");
  }
  return (num_bytes);
}

static size_t process_bag(
  const std::string & inFile, const std::string & outFile, const std::string & topic,
  const std::string & header)
{
  std::fstream out;
  out.open(outFile, std::ios::out | std::ios::binary);
  out.write(header.c_str(), header.size());
  size_t numBytes(0);
  {
    rosbag2_cpp::Reader reader;
    reader.open(inFile);
    rclcpp::Serialization<EventPacket> serialization;
    uint32_t last_evt_stamp(0);
    while (reader.has_next()) {
      auto msg = reader.read_next();
      if (msg->topic_name == topic) {
        rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
        EventPacket m;
        serialization.deserialize_message(&serializedMsg, &m);
        numBytes +=
          write(out, &m.events[0], m.events.size(), m.time_base, m.encoding, &last_evt_stamp);
      }
    }
  }  // close reader when out of scope
  return (numBytes);
}
}  // namespace event_camera_tools

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outFile;
  std::string camera;
  std::string topic("/event_camera/events");
  while ((opt = getopt(argc, argv, "b:o:t:c:h")) != -1) {
    switch (opt) {
      case 'b':
        inFile = optarg;
        break;
      case 'o':
        outFile = optarg;
        break;
      case 't':
        topic = optarg;
        break;
      case 'c':
        camera = optarg;
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
  if (inFile.empty() || outFile.empty()) {
    std::cout << "missing input or output file name!" << std::endl;
    usage();
    return (-1);
  }
  const auto h = event_camera_tools::headers.find(camera);
  if (camera.empty() || h == event_camera_tools::headers.end()) {
    std::cout << "missing or unknown camera, must specify one of these: " << std::endl;
    for (const auto & h : event_camera_tools::headers) {
      std::cout << "  " << h.first << std::endl;
    }
    return (-1);
  }

  auto start = std::chrono::high_resolution_clock::now();
  const size_t numBytes = event_camera_tools::process_bag(inFile, outFile, topic, h->second);
  auto final = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  std::cout << "number of bytes read: " << numBytes * 1e-6 << " MB in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "bytes rate: " << static_cast<double>(numBytes) / total_duration.count() << " MB/s"
            << std::endl;
  return (0);
}
