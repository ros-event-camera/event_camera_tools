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

#ifndef EVENT_CAMERA_TOOLS__MOVIE_MAKER_H_
#define EVENT_CAMERA_TOOLS__MOVIE_MAKER_H_

#include <event_camera_codecs/event_processor.h>

#include <fstream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace event_camera_tools
{
class MovieMaker : public event_camera_codecs::EventProcessor
{
public:
  ~MovieMaker() override {}
  // ---------- from the EventProcessor interface:
  void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    if (nextFrameTime_ == 0) {
      nextFrameTime_ = sensor_time + framePeriod_;
    }
    auto cur = img_.at<cv::Vec3b>(ey, ex);
    cur[polarity ? 0 : 2] = 255;
    img_.at<cv::Vec3b>(ey, ex) = cur;
    while (sensor_time > nextFrameTime_) {
      writeImage(frameNum_++, nextFrameTime_, img_);
      resetImage(img_.cols, img_.rows);
      nextFrameTime_ += framePeriod_;
    }
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return (true); }
  void finished() override { isFirst_ = true; }
  void rawData(const char *, size_t) override {}
  // ----------- end of inherited
  void setOutputDir(const std::string & dir) { outputDir_ = dir; }
  void setTimeStampsFile(const std::string & f) { timeStampsFile_ = f; }
  void resetImage(uint32_t w, uint32_t h) { img_ = cv::Mat::zeros(h, w, CV_8UC3); }
  void setFramePeriod(double sec) { framePeriod_ = static_cast<uint64_t>(1e9 * sec); }

private:
  void writeImage(size_t frameNum, uint64_t sensorTime, const cv::Mat & img)
  {
    char fname[256];
    fname[sizeof(fname) - 1] = 0;
    snprintf(fname, sizeof(fname) - 1, "%s/frame_%05zu.png", outputDir_.c_str(), frameNum);
    cv::imwrite(fname, img);
    writeTimeStamp(frameNum, sensorTime);
    std::cout << "writing to: " << fname << " mean: " << cv::mean(img) << std::endl;
  }
  void writeTimeStamp(size_t frameNum, uint64_t sensorTime)
  {
    if (timeStampsFile_.empty()) {
      return;
    }
    if (!timeStampsStream_.is_open()) {
      timeStampsStream_.open(timeStampsFile_);
      timeStampsStream_ << std::setw(12) << "frame" << std::setw(24) << "sensor_time" << '\n';
    }
    timeStampsStream_ << std::setw(12) << frameNum << std::setw(24) << sensorTime << '\n';
  }
  // --------------- variables -----------------
  cv::Mat img_;
  uint64_t framePeriod_{0};
  std::string outputDir_{"."};
  std::string timeStampsFile_;
  std::ofstream timeStampsStream_;
  uint64_t nextFrameTime_{0};
  size_t frameNum_{0};
  bool isFirst_{true};
};
}  // namespace event_camera_tools
#endif  // EVENT_CAMERA_TOOLS__MOVIE_MAKER_H_
