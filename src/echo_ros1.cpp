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

#include <event_array_codecs/decoder.h>
#include <event_array_codecs/event_processor.h>
#include <event_array_msgs/EventArray.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <unordered_map>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "echo <ros_topic>" << std::endl;
}

using event_array_codecs::Decoder;
using event_array_msgs::EventArray;

class Echo : public event_array_codecs::EventProcessor
{
public:
  explicit Echo(const std::string & topic, const ros::NodeHandle & nh) : nh_(nh)
  {
    ROS_INFO_STREAM("subscribing to " << topic);
    sub_ = nh_.subscribe(topic, 1, &Echo::eventMsg, this);
  }
  // ---------- from the EventProcessor interface:
  void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    printf("%8" PRIu64 " %4d %4d %1d\n", sensor_time, ex, ey, polarity);
  }
  void eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t id) override
  {
    printf("%8" PRIu64 " edge: %1d  id: %2d\n", sensor_time, edge, id);
  }
  void finished() override {}
  void rawData(const char *, size_t) override {}
  // -------- end of inherited

  void eventMsg(EventArray::ConstPtr msg)
  {
    printf("-------------------------------\n");
    printf("res: %4d  height: %4d enc: %s\n", msg->width, msg->height, msg->encoding.c_str());
    printf("header stamp: %8" PRIu64 "\n", msg->header.stamp.toNSec());
    printf("time base: %8" PRIu64 "\n", msg->time_base);
    printf("seqno: %8" PRIu64 "\n", msg->seq);
    printf("---\n");
    auto decIt = decoders_.find(msg->encoding);
    if (decIt == decoders_.end()) {
      auto dec = Decoder::newInstance(msg->encoding);
      if (dec) {
        decIt = decoders_.insert({msg->encoding, dec}).first;
      } else {
        printf("unsupported encoding: %s\n", msg->encoding.c_str());
        return;
      }
    }
    auto & decoder = *(decIt->second);
    decoder.setTimeBase(msg->time_base);
    decoder.decode(&msg->events[0], msg->events.size(), this);
  }
  // ---------- variables
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::unordered_map<std::string, std::shared_ptr<Decoder>> decoders_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "echo");
  ros::NodeHandle pnh("~");

  std::string topic;
  switch (argc) {
    case 2:
      topic = argv[1];
      break;
    default:
      usage();
      exit(-1);
  }
  try {
    Echo node(topic, pnh);
    ros::spin();
  } catch (const std::exception & e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
    return (-1);
  }
  return (0);
}
