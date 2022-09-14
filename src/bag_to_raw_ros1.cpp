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

#include <event_array_msgs/EventArray.h>
#include <event_array_msgs/decode.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <map>

#include "evt_3_utils.h"

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_raw -b name_of_bag_file -o name_of_raw_file -t topic -c camera" << std::endl;
}

namespace event_array_tools
{
using event_array_msgs::EventArray;

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

static size_t process_bag(
  const std::string & inFile, const std::string & outFile, const std::string & topic,
  const std::string & header)
{
  std::fstream out;
  out.open(outFile, std::ios::out | std::ios::binary);
  out.write(header.c_str(), header.size());
  size_t numMessages(0);
  uint32_t last_evt_stamp(0);

  {
    rosbag::Bag bag;
    std::cout << "reading from bag: " << inFile << " topic: " << topic << std::endl;
    bag.open(inFile, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery({topic}));

    for (const rosbag::MessageInstance & m : view) {
      if (m.getTopic() == topic) {
        EventArray::ConstPtr ea = m.instantiate<EventArray>();
        if (ea) {
          if (ea->encoding == "evt3") {
            out.write(reinterpret_cast<const char *>(&ea->events[0]), ea->events.size());
          } else if (ea->encoding == "mono") {
            (void)evt_3_utils::write(
              out, &ea->events[0], ea->events.size(), ea->time_base, ea->encoding, &last_evt_stamp);
          }
          numMessages++;
        }
      }
    }
    bag.close();
  }
  std::cout << "read " << numMessages << " messages" << std::endl;
  return (numMessages);
}  // namespace event_array_tools
}  // namespace event_array_tools

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outFile;
  std::string topic("/event_camera/packets");
  std::string camera("silkyev");
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
  const auto h = event_array_tools::headers.find(camera);
  if (h == event_array_tools::headers.end()) {
    std::cout << "Unknown camera! Supported are: " << std::endl;
    for (const auto & c : event_array_tools::headers) {
      std::cout << c.first << std::endl;
    }
  }
  auto start = std::chrono::high_resolution_clock::now();
  const size_t numMsgs = event_array_tools::process_bag(inFile, outFile, topic, h->second);
  auto final = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  std::cout << "message rate: " << static_cast<double>(numMsgs) * 1e6 / total_duration.count()
            << " 1/s" << std::endl;

  return (0);
}
