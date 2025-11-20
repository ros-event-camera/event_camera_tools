// -*-c++-*--------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <assert.h>
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_tools/check_endian.h>
#include <event_camera_tools/ros_compat.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "raw_to_bag -b name_of_bag_file -i name_of_raw_file -t topic -f "
               "frame_id -w width -h height [-p packet_period_ms] [-T start_time(UTC sec)]"
            << std::endl;
}
namespace event_camera_tools
{
using event_camera_codecs::EventPacket;

class MessageUpdaterEvt3 : public event_camera_codecs::EventProcessor
{
public:
  explicit MessageUpdaterEvt3(
    const std::string & bagName, const std::string & topic, const std::string & frameId,
    uint32_t width, uint32_t height, uint64_t framePeriod, ros_compat::Time initialTime)
  : topic_(topic), startRosTime_(initialTime), framePeriod_(framePeriod)
  {
    writer_ = std::make_unique<ros_compat::Writer>();
#ifdef USING_ROS_1
    writer_->open(bagName, rosbag::bagmode::Write);
#else
    writer_->open(bagName);
#endif
    createTopic(topic);
    msg_.header.frame_id = frameId;
    msg_.width = width;
    msg_.height = height;
    msg_.encoding = "evt3";
    msg_.is_bigendian = check_endian::isBigEndian();
    msg_.seq = 0;
    decoder_ = decoderFactory_.getInstance("evt3", width, height);
    if (!decoder_) {
      std::cerr << "evt3 not supported for decoding!" << std::endl;
      throw(std::runtime_error("evt3 not supported!"));
    }
  }
  void createTopic(const std::string & topic)
  {
#ifdef USING_ROS_1
    (void)topic;
#else
    struct rosbag2_storage::TopicMetadata md;
    md.name = topic;
    md.type = "event_camera_msgs/msg/EventPacket";
    md.serialization_format = rmw_get_serialization_format();
    writer_->create_topic(md);
#endif
  }
  void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    numEvents_[polarity]++;
    (void)sensor_time;
    (void)ex;
    (void)ey;
    (void)polarity;
  }

  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return (true); }
  void finished() override {}
  void rawData(const char *, size_t) override {}

  ~MessageUpdaterEvt3() { writer_.reset(); }

  bool findFirstSensorTime(const uint8_t * data, size_t len)
  {
    auto decoder = decoderFactory_.newInstance("evt3");
    decoder->setGeometry(msg_.width, msg_.height);
    return (decoder->findFirstSensorTime(data, len, &firstSensorTime_));
  }

  size_t writeMessage(const uint8_t * buffer, const size_t oldOffset, uint64_t frameTime)
  {
    const size_t newOffset = decoder_->getNumberOfBytesUsed();
    if (newOffset > oldOffset) {
      msg_.events.insert(msg_.events.end(), buffer + oldOffset, buffer + newOffset);
      doWrite();
      msg_.events.clear();
      msg_.seq++;
      // set the header stamp for the next message based on what time we have now.
      msg_.header.stamp =
        startRosTime_ + ros_compat::duration_from_nanoseconds(frameTime - firstSensorTime_);
      headerSensorTime_ = frameTime;
    }
    return (newOffset);
  }
  void doWrite()
  {
#ifdef USING_ROS_1
    writer_->write(topic_, msg_.header.stamp, msg_);
#else
    rclcpp::Serialization<EventPacket> serialization;
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    serialization.serialize_message(&msg_, serialized_msg.get());
    writer_->write(
      serialized_msg, topic_, "event_camera_msgs/msg/EventPacket",
      ros_compat::Time(msg_.header.stamp));
#endif
  }

  void processRawData(const uint8_t * data, size_t len)
  {
    if (!hasValidSensorTime_) {
      if (!findFirstSensorTime(data, len)) {
        std::cout << "WARNING: packet skipped because no event time found!" << std::endl;
        return;
      }
      hasValidSensorTime_ = true;
      currentTime_ = firstSensorTime_;
      msg_.header.stamp = startRosTime_;
      nextFrameTime_ = currentTime_ + framePeriod_;
      std::cout << "start ros time: " << ros_compat::to_nanoseconds(startRosTime_)
                << " first sensor time: " << firstSensorTime_ << std::endl;
    }
    for (size_t offset = 0; offset < len;) {  // must consume entire buffer
      uint64_t nextTime{0};
      const bool timeLimitReached =
        decoder_->decodeUntil(data, len, this, nextFrameTime_, 0ULL, &nextTime);
      if (!timeLimitReached) {  // buffer completely consumed!
        // add data buffer to current message and be done
        msg_.events.insert(msg_.events.end(), data + offset, data + len);
        return;
      } else {
        currentTime_ = nextTime;  // only valid when time limit is reached!
        offset = writeMessage(data, offset, currentTime_);
        nextFrameTime_ += framePeriod_;
      }
    }
    throw(std::runtime_error("internal bug: should not reach this location ever!"));
  }
  const size_t * getNumEvents() const { return (numEvents_); }

private:
  std::unique_ptr<ros_compat::Writer> writer_;
  EventPacket msg_;
  std::string topic_;
  size_t numEvents_[2]{0, 0};
  event_camera_codecs::DecoderFactory<EventPacket, MessageUpdaterEvt3> decoderFactory_;
  event_camera_codecs::Decoder<EventPacket, MessageUpdaterEvt3> * decoder_;
  bool hasValidRosTime_{false};
  ros_compat::Time startRosTime_;
  bool hasValidSensorTime_{false};
  uint64_t currentTime_{0};
  uint64_t firstSensorTime_{0};
  uint64_t nextFrameTime_{0};
  uint64_t framePeriod_{0};
  uint64_t headerSensorTime_{0};  // XXX debug
};

static void skip_header(std::fstream & in)
{
  int c;
  while ((c = in.peek()) == '%') {
    std::string line;
    std::getline(in, line);
  }
}

}  // namespace event_camera_tools

int main(int argc, char ** argv)
{
  int opt;
#ifdef USING_ROS_1
  ros::Time::init();
#endif
  std::string inFile;
  std::string outFile;
  std::string topic("/event_camera/events");
  std::string frameId("event_camera");
  int height(0);
  int width(0);
  const int bufSize(1000000);
  double packetDurationMillis(1);
  double startTimeSec{-1.0};
  while ((opt = getopt(argc, argv, "b:i:t:f:h:w:T:p:")) != -1) {
    switch (opt) {
      case 'b':
        outFile = optarg;
        break;
      case 'i':
        inFile = optarg;
        break;
      case 'p':
        packetDurationMillis = std::stod(optarg);
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
      case 'T':
        startTimeSec = std::stod(optarg);
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
  if (height == 0 || width == 0) {
    std::cout << "missing geometry, must specify height and width!" << std::endl;
    usage();
    return (-1);
  }
  const auto start = std::chrono::high_resolution_clock::now();
  const uint64_t framePeriod = static_cast<uint64_t>(packetDurationMillis * 1000000);
  event_camera_tools::MessageUpdaterEvt3 updater(
    outFile, topic, frameId, width, height, framePeriod,
    startTimeSec < 0 ? ros_compat::now() : ros_compat::time_from_sec(startTimeSec));
  std::fstream in;
  in.open(inFile, std::ios::in | std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "cannot open file: " << inFile << std::endl;
    return (-1);
  }
  event_camera_tools::skip_header(in);
  std::vector<char> buffer(bufSize);
  size_t bytesRead(0);
  while (true) {
    in.read(&(buffer[0]), buffer.size());
    if (in.gcount() != 0) {
      updater.processRawData(reinterpret_cast<const uint8_t *>(&buffer[0]), in.gcount());
    }
    bytesRead += in.gcount();
    if (in.gcount() < std::streamsize(buffer.size())) {
      break;
    }
  }

  const decltype(start) final = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  const size_t numEvents = updater.getNumEvents()[0] + updater.getNumEvents()[1];
  std::cout << "number of bytes read: " << bytesRead / (1024 * 1024) << " MB in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "total number of events: " << numEvents << std::endl;
  std::cout << "byte rate: " << static_cast<double>(bytesRead) / total_duration.count() << " MB/s"
            << std::endl;
  std::cout << "event rate: " << static_cast<double>(numEvents) / total_duration.count() << " Mev/s"
            << std::endl;
  return (0);
}
