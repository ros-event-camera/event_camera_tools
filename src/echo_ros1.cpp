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
#include <event_camera_msgs/EventPacket.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <unordered_map>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "echo [-b bag] <ros_topic>" << std::endl;
}

using event_camera_codecs::Decoder;
using event_camera_codecs::EventPacket;

class Echo : public event_camera_codecs::EventProcessor
{
public:
  // ---------- from the EventProcessor interface:
  void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    printf("%8" PRIu64 " %4d %4d %1d\n", sensor_time, ex, ey, polarity);
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

  void eventMsg(EventPacket::ConstPtr msg)
  {
    printf("-------------------------------\n");
    printf("width: %4d  height: %4d enc: %s\n", msg->width, msg->height, msg->encoding.c_str());
    printf("header stamp: %8" PRIu64 "\n", ros::Time(msg->header.stamp).toNSec());
    printf("time base: %8" PRIu64 "\n", msg->time_base);
    printf("seqno: %8" PRIu64 "\n", msg->seq);
    printf("---\n");
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
};

class EchoNode
{
public:
  explicit EchoNode(const std::string & topic, const ros::NodeHandle & nh, Echo * echo)
  : nh_(nh), echo_(echo)
  {
    ROS_INFO_STREAM("subscribing to " << topic);
    sub_ = nh_.subscribe(topic, 1, &Echo::eventMsg, echo_);
  }
  void eventMsg(EventPacket::ConstPtr msg) { echo_->eventMsg(msg); }
  // ---------- variables
  ros::NodeHandle nh_;
  Echo * echo_;
  ros::Subscriber sub_;
};

static void read_from_bag(const std::string & bagName, const std::string & topic, Echo * echo)
{
  rosbag::Bag bag;
  std::cout << "reading from bag: " << bagName << " topic: " << topic << std::endl;
  bag.open(bagName, rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery({topic}));
  size_t msgCount(0);
  for (const rosbag::MessageInstance & m : view) {
    if (m.getTopic() == topic) {
      EventPacket::ConstPtr ea = m.instantiate<EventPacket>();
      if (ea) {
        echo->eventMsg(ea);
        msgCount++;
      }
    }
  }
  std::cout << "found " << msgCount << " messages in bag" << std::endl;
}

int main(int argc, char ** argv)
{
  int opt;

  std::string topic;
  std::string bagFile;
  ros::init(argc, argv, "echo");
  while ((opt = getopt(argc, argv, "b:")) != -1) {
    switch (opt) {
      case 'b':
        bagFile = optarg;
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
  if (optind != argc - 1) {
    std::cout << "expected topic!" << std::endl;
    usage();
    return (-1);
  }
  topic = argv[optind];

  Echo echo;
  if (bagFile.empty()) {
    ros::NodeHandle pnh("~");
    try {
      EchoNode node(topic, pnh, &echo);
      ros::spin();
    } catch (const std::exception & e) {
      ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
      return (-1);
    }
  } else {
    read_from_bag(bagFile, topic, &echo);
    size_t totCDEvents = echo.getNumCDEvents()[0] + echo.getNumCDEvents()[1];
    size_t totTrigEvents = echo.getNumTrigEvents()[0] + echo.getNumTrigEvents()[1];
    std::cout << "cd      ev: " << totCDEvents << " time: " << echo.getCDStamps()[0] << " -> "
              << echo.getCDStamps()[1] << std::endl;
    std::cout << "trigger ev: " << totTrigEvents << " time: " << echo.getTrigStamps()[0] << " -> "
              << echo.getTrigStamps()[1] << std::endl;
  }

  return (0);
}
