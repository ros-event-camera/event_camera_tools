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

#include <fstream>
#include <vector>

namespace event_camera_tools
{
class EventStatistics : public event_camera_codecs::EventProcessor
{
public:
  explicit EventStatistics(const std::string & fname) { file_.open(fname); }
  ~EventStatistics()
  {
    writeCount();
    file_.close();
  }
  // ---------- from the EventProcessor interface:
  void eventCD(uint64_t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    count_[polarity][ey * width_ + ex]++;
    num_events_++;
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return (true); }
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
  const auto getNumEvents() const { return (num_events_); }

private:
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
  // --------------- variables -----------------
  std::vector<uint64_t> count_[2];
  std::ofstream file_;
  uint32_t width_{0};
  uint32_t height_{0};
  size_t num_events_{0};
};
}  // namespace event_camera_tools
#endif  // EVENT_CAMERA_TOOLS__EVENT_STATISTICS_H_
