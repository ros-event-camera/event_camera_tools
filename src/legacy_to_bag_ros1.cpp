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

#include <dvs_msgs/EventArray.h>
#include <event_camera_codecs/encoder.h>
#include <event_camera_msgs/EventPacket.h>
#include <prophesee_event_msgs/EventArray.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unistd.h>

#include <chrono>
#include <iostream>

#include "event_camera_tools/check_endian.h"

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "legacy_to_bag -b name_of_legacy_bag_file -o name_of_new_bag -t topic " << std::endl;
}

namespace event_camera_tools
{
template <typename MsgType>
static size_t processMsg(
  const typename MsgType::ConstPtr & inMsg, const std::string & topic, rosbag::Bag * outBag,
  event_camera_codecs::Encoder * enc)
{
  if (inMsg->events.empty()) {
    return (0);  // drop empty messages
  }
  event_camera_msgs::EventPacket outMsg;
  outMsg.header = inMsg->header;
  enc->setBuffer(&outMsg.events);
  // use first event time to set sensor time
  const ros::Time firstEventTime = inMsg->events[0].ts;
  outMsg.time_base = firstEventTime.toNSec();
  outMsg.height = inMsg->height;
  outMsg.width = inMsg->width;
  outMsg.encoding = "evt3";
  outMsg.seq = inMsg->header.seq;
  outMsg.is_bigendian = check_endian::isBigEndian();
  enc->setSensorTime(firstEventTime.toNSec());
  // use encoder to convert events
  for (const auto & e : inMsg->events) {
    enc->encodeCD((e.ts - firstEventTime).toNSec(), e.x, e.y, e.polarity);
  }
  enc->flush();
  outBag->write(topic, inMsg->header.stamp, outMsg);
  return (inMsg->events.size());
}

static size_t process_bag(
  const std::string & inBagName, const std::string & outBagName, const std::string & topic)
{
  std::cout << "reading from bag: " << inBagName << " topic: " << topic << std::endl;
  std::cout << "writing to bag: " << outBagName << std::endl;
  rosbag::Bag inBag;
  inBag.open(inBagName, rosbag::bagmode::Read);
  rosbag::Bag outBag;
  outBag.open(outBagName, rosbag::bagmode::Write);
  auto encoder = event_camera_codecs::Encoder::newInstance("evt3");
  if (!encoder) {
    std::cerr << "evt3 codec not supported by encoder!" << std::endl;
    throw(std::runtime_error("evt3 unsupported!"));
  }
  rosbag::View view(inBag, rosbag::TopicQuery({topic}));
  size_t numMessages(0);
  size_t numEvents(0);
  for (const rosbag::MessageInstance & m : view) {
    if (m.getTopic() == topic) {
      dvs_msgs::EventArray::ConstPtr dvs = m.instantiate<dvs_msgs::EventArray>();
      if (dvs) {
        numEvents += processMsg<dvs_msgs::EventArray>(dvs, topic, &outBag, encoder.get());
        numMessages++;
      } else {
        prophesee_event_msgs::EventArray::ConstPtr proph =
          m.instantiate<prophesee_event_msgs::EventArray>();
        if (proph) {
          numEvents +=
            processMsg<prophesee_event_msgs::EventArray>(proph, topic, &outBag, encoder.get());
          numMessages++;
        }
      }
    }
  }
  inBag.close();
  outBag.close();
  std::cout << "read " << numMessages << " messages" << std::endl;
  return (numEvents);
}
}  // namespace event_camera_tools

int main(int argc, char ** argv)
{
  int opt;
  ros::Time::init();
  std::string inBag;
  std::string outBag;
  std::string topic("/event_camera/events");
  while ((opt = getopt(argc, argv, "b:o:t:")) != -1) {
    switch (opt) {
      case 'b':
        inBag = optarg;
        break;
      case 'o':
        outBag = optarg;
        break;
      case 't':
        topic = optarg;
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (inBag.empty() || outBag.empty()) {
    std::cout << "missing input or output bag name!" << std::endl;
    usage();
    return (-1);
  }
  const auto start = std::chrono::high_resolution_clock::now();

  const size_t numEvents = event_camera_tools::process_bag(inBag, outBag, topic);

  const auto end = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  std::cout << "total number of events: " << numEvents << std::endl;
  std::cout << "event rate: " << static_cast<double>(numEvents) / total_duration.count() << " Mev/s"
            << std::endl;

  return (0);
}
