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

#include <assert.h>
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_tools/check_endian.h>
#include <unistd.h>

#include <chrono>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "raw_to_bag -b name_of_bag_file -i name_of_raw_file -t topic -f "
               "frame_id -w width -h height -B buf_size [-T start_time(UTC sec)]"
            << std::endl;
}

namespace event_camera_tools
{
using event_camera_codecs::EventPacket;

class MessageUpdaterEvt3
{
public:
  explicit MessageUpdaterEvt3(
    const std::string & bagName, const std::string & topic, const std::string & frameId,
    uint32_t width, uint32_t height, rclcpp::Time initialTime)
  : topic_(topic), initialRosTime_(initialTime)
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(bagName);
    struct rosbag2_storage::TopicMetadata md;
    md.name = topic;
    md.type = "event_camera_msgs/msg/EventPacket";
    md.serialization_format = rmw_get_serialization_format();
    md.offered_qos_profiles = "";
    // md.type_description_hash = "";
    writer_->create_topic(md);
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

  ~MessageUpdaterEvt3() { writer_.reset(); }

  void processRawData(const char * data, size_t len)
  {
    uint64_t sensorTime(0);
    const bool hasValidSensorTime = decoder_->summarize(
      reinterpret_cast<const uint8_t *>(data), len, &sensorTime, &lastSensorTime_, numEvents_);

    if (!hasValidRosTime_) {
      startRosTime_ = initialRosTime_;
      if (hasValidSensorTime) {
        startSensorTime_ = sensorTime;
        hasValidRosTime_ = true;
      } else {
        std::cout << "skipping raw packet at beginning without time stamp!" << std::endl;
        return;
      }
    }
    msg_.header.stamp =
      startRosTime_ + rclcpp::Duration(std::chrono::nanoseconds(sensorTime - startSensorTime_));
    msg_.events.clear();  // remove marker
    msg_.events.insert(msg_.events.end(), data, data + len);

    rclcpp::Serialization<EventPacket> serialization;
#ifdef USE_OLD_ROSBAG_API
    rclcpp::SerializedMessage serialized_msg;
    serialization.serialize_message(&msg_, &serialized_msg);
#else
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    serialization.serialize_message(&msg_, serialized_msg.get());
#endif
    writer_->write(
      serialized_msg, topic_, "event_camera_msgs/msg/EventPacket", rclcpp::Time(msg_.header.stamp));
    msg_.events.clear();  // mark as new
    msg_.seq++;
  }
  const size_t * getNumEvents() const { return (numEvents_); }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  EventPacket msg_;
  std::string topic_;
  size_t numEvents_[2]{0, 0};
  event_camera_codecs::DecoderFactory<EventPacket> decoderFactory_;
  event_camera_codecs::Decoder<EventPacket> * decoder_;
  bool hasValidRosTime_{false};
  rclcpp::Time startRosTime_;
  rclcpp::Time initialRosTime_;
  uint64_t startSensorTime_{0};
  uint64_t lastSensorTime_{0};
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

  std::string inFile;
  std::string outFile;
  std::string topic("/event_camera/events");
  std::string frameId("event_camera");
  int height(0);
  int width(0);
  int bufSize(150000);
  double startTimeSec{-1.0};
  while ((opt = getopt(argc, argv, "b:i:t:f:h:w:B:T:")) != -1) {
    switch (opt) {
      case 'b':
        outFile = optarg;
        break;
      case 'i':
        inFile = optarg;
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
      case 'B':
        bufSize = 2 * atoi(optarg);  // event has 2 bytes!
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

  event_camera_tools::MessageUpdaterEvt3 updater(
    outFile, topic, frameId, width, height,
    startTimeSec < 0 ? rclcpp::Clock().now()
                     : rclcpp::Time(static_cast<uint64_t>(startTimeSec * 1e9)));
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
      updater.processRawData(reinterpret_cast<char *>(&buffer[0]), in.gcount());
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
