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

#ifndef EVENT_CAMERA_TOOLS__EVENT_STATISTICS_H_
#define EVENT_CAMERA_TOOLS__EVENT_STATISTICS_H_
#include <event_camera_codecs/event_processor.h>

#include <cstdint>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

namespace event_camera_tools
{
class EventStatistics : public event_camera_codecs::EventProcessor
{
public:
  using TimePair = std::pair<uint64_t, uint64_t>;
  explicit EventStatistics(const std::string & fname) { file_.open(fname); }
  ~EventStatistics()
  {
    writeCount();
    printStatistics();
    file_.close();
  }
  // ---------- from the EventProcessor interface:
  void eventCD(uint64_t t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    count_[polarity][ey * width_ + ex]++;
    updateTimeStatistics(t);
    num_events_++;
  }
  bool eventExtTrigger(uint64_t t, uint8_t edge, uint8_t) override
  {
    num_trigger_events_[edge]++;
    updateTimeStatistics(t);
    return (true);
  }
  void finished() override {}
  void rawData(const char *, size_t) override {}
  // ----------- end of inherited
  void initialize(uint32_t w, uint32_t h)
  {
    if (w == 0 || h == 0) {
      throw std::invalid_argument("height and width must be non-zero");
    }
    width_ = w;
    height_ = h;
    count_[0].resize(w * h, 0);
    count_[1].resize(w * h, 0);
  }
  void setMaxDeltaTime(uint64_t dt) { max_delta_time_ = dt; }
  const auto getNumEvents() const { return (num_events_); }
  const auto & getBackwardTimes() const { return (backward_times_); }
  const auto & getJumpTimes() const { return (jump_times_); }
  int64_t getTimePassed() const { return (last_time_ - first_time_); }

private:
  rclcpp::Logger logger() { return (rclcpp::get_logger("event_statistics")); }
  void updateTimeStatistics(uint64_t t)
  {
    if (first_time_ == 0) {
      first_time_ = t;
    }
    if (t < last_time_) {
      backward_times_.push_back({last_time_, t});
    }
    if (t > last_time_ + max_delta_time_) {
      if (last_time_ != 0) {
        jump_times_.push_back({last_time_, t});
      }
    }
    last_time_ = t;
  }

  void writeCount()
  {
    if (num_events_ == 0) {
      throw std::runtime_error("no events found for topic!");
    }
    if (width_ == 0 || height_ == 0) {
      throw std::invalid_argument("cannot write, height and width must be non-zero");
    }

    for (size_t i = 0; i < height_; i++) {
      for (size_t j = 0; j < width_ - 1; j++) {
        const size_t idx = i * width_ + j;
        file_ << " " << count_[0][idx] << " " << count_[1][idx];
      }
      const size_t idx = (i + 1) * width_ - 1;
      file_ << " " << count_[0][idx] << " " << count_[1][idx] << std::endl;
    }
  }
  void printStatistics()
  {
    for (const auto & jump : jump_times_) {
      std::cout << " jump: " << jump.first << " -> " << jump.second
                << " dt: " << (jump.second - jump.first) * 1e-9 << std::endl;
    }
    RCLCPP_INFO_STREAM(logger(), "jump max delta time: " << max_delta_time_ * 1e-9);
    RCLCPP_INFO_STREAM(logger(), "number of jumps: " << jump_times_.size());
    RCLCPP_INFO_STREAM(logger(), "number of backward times: " << backward_times_.size());
    RCLCPP_INFO_STREAM(logger(), "total number of cd events: " << num_events_);
    RCLCPP_INFO_STREAM(
      logger(),
      "trigger events UP: " << num_trigger_events_[0] << ", DOWN: " << num_trigger_events_[1]);
  }
  // --------------- variables -----------------
  std::vector<uint64_t> count_[2];
  std::ofstream file_;
  uint32_t width_{0};
  uint32_t height_{0};
  size_t num_events_{0};
  size_t num_trigger_events_[2]{0, 0};
  uint64_t first_time_{0};
  uint64_t last_time_{0};
  uint64_t max_delta_time_{1000000};
  std::vector<TimePair> jump_times_;
  std::vector<TimePair> backward_times_;
};
}  // namespace event_camera_tools
#endif  // EVENT_CAMERA_TOOLS__EVENT_STATISTICS_H_
