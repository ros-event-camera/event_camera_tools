// -*-c++-*--------------------------------------------------------------------
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

#include <chrono>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "plot_events -b name_of_bag_file -o name_of_plot_file -t topic" << std::endl;
}

namespace event_camera_tools
{
class Plotter : public event_camera_codecs::EventProcessor
{
public:
  explicit Plotter(const std::string & fname) { file_.open(fname, std::ios::out); }
  ~Plotter() { file_.close(); }
  void eventCD(uint64_t t, uint16_t ex, uint16_t ey, uint8_t p) override
  {
    (void)ex;
    (void)ey;
    (void)p;
    if (t0_ == 0) {
      t0_ = t;
    } else {
      int64_t dt = t - lastTime_;
      if (dt < 0) {
        std::cerr << "negative time delta!" << std::endl;
        std::cerr << "now:    " << t << std::endl;
        std::cerr << "before: " << lastTime_ << std::endl;
        // throw std::runtime_error("negative time difference!");
      }
      if (dt > dtMax_) {
        std::cout << "max time difference: " << dt << " at time: " << lastTime_ << std::endl;
        dtMax_ = dt;
      }
    }
    file_ << (t - t0_) / 1000
          << std::endl;  // " " << ex << " " << ey << " " << static_cast<int>(p) << std::endl;
    numEvents_++;
    lastTime_ = t;
  }
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}
  void finished() override{};
  void rawData(const char *, size_t) override{};
  size_t getNumEvents() const { return (numEvents_); }

private:
  std::ofstream file_;
  uint64_t t0_{0};
  size_t numEvents_{0};
  int64_t dtMax_{0};
  uint64_t lastTime_{0};
};

using event_camera_msgs::msg::EventPacket;
static size_t process_bag(
  const std::string & inFile, const std::string & outFile, const std::string & topic,
  size_t * numEvents, size_t * numMessages)
{
  Plotter plotter(outFile);
  size_t numBytes(0);
  *numMessages = 0;
  rosbag2_cpp::Reader reader;
  reader.open(inFile);
  rclcpp::Serialization<EventPacket> serialization;
  event_camera_codecs::DecoderFactory<EventPacket, Plotter> factory;

  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (msg->topic_name == topic) {
      rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
      EventPacket m;
      serialization.deserialize_message(&serializedMsg, &m);
      auto decoder = factory.getInstance(m);
      decoder->decode(m, &plotter);
      numBytes += m.events.size();
      (*numMessages)++;
    }
  }
  *numEvents = plotter.getNumEvents();
  return (numBytes);
}
}  // namespace event_camera_tools

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outFile;
  std::string topic("/event_camera/events");
  while ((opt = getopt(argc, argv, "b:o:t:h")) != -1) {
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
  auto start = std::chrono::high_resolution_clock::now();
  size_t numEvents{0};
  size_t numMessages{0};
  size_t numBytes{0};

  try {
    numBytes = event_camera_tools::process_bag(inFile, outFile, topic, &numEvents, &numMessages);
  } catch (const std::runtime_error & e) {
    std::cout << "aborted due to exception: " << e.what() << std::endl;
  }
  auto final = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  std::cout << "number of bytes read: " << numBytes * 1e-6 << " MB in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "bytes rate: " << static_cast<double>(numBytes) / total_duration.count() << " MB/s"
            << std::endl;
  std::cout << "event rate: " << static_cast<double>(numEvents) / total_duration.count() << " Mev/s"
            << std::endl;
  std::cout << "message rate: " << static_cast<double>(numMessages) / total_duration.count()
            << " msgs/s" << std::endl;
  std::cout << "events per message: " << numEvents / std::max<size_t>(numMessages, 1ull)
            << std::endl;
  return (0);
}
