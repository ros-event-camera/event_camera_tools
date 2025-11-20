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

#ifndef EVENT_CAMERA_TOOLS__TIME_SLICER_H_
#define EVENT_CAMERA_TOOLS__TIME_SLICER_H_

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/event_processor.h>
#include <event_camera_tools/check_endian.h>
#include <event_camera_tools/frame_time.h>
#include <inttypes.h>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#undef NDEBUG
#include <cassert>

namespace event_camera_tools
{
using event_camera_codecs::Decoder;
using event_camera_msgs::msg::EventPacket;

template <typename Processor>
class TimeSlicer : public event_camera_codecs::EventProcessor
{
public:
  TimeSlicer() = default;
  // ---------- inherited from EventProcessor
  inline void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    processor_->eventCD(sensor_time, ex, ey, polarity);
  }
  bool eventExtTrigger(uint64_t t, uint8_t edge, uint8_t id) override
  {
    return (processor_->eventExtTrigger(t, edge, id));
  }
  void finished() override { processor_->finished(); };
  void rawData(const char * buf, size_t sz) override { processor_->rawData(buf, sz); };
  // --------- end of inherited from EventProcessor

  void setProcessor(Processor * p) { processor_ = p; }
  void setEventQueueMemoryLimit(size_t lim) { event_queue_memory_limit_ = lim; }
  void setSensorPeriod(int64_t period)
  {
    if (ros_period_ != rclcpp::Duration(0, 0)) {
      RCLCPP_WARN(logger_, "already set ROS time period, ignoring sensor period!");
    } else {
      sensor_period_ = period;
    }
  }
  void setROSPeriod(const rclcpp::Duration & period)
  {
    if (sensor_period_ != 0) {
      RCLCPP_WARN(logger_, "already set sensor time period, ignoring ROS period!");
    } else {
      ros_period_ = period;
    }
  }
  void setLogger(rclcpp::Logger logger) { logger_ = logger; }

  void eventMsg(typename EventPacket::ConstSharedPtr msg)
  {
    if (t0_ == std::numeric_limits<int64_t>::lowest() && !handleFirstMessage(msg)) {
      return;
    }
    if (event_queue_memory_ + msg->events.size() < static_cast<size_t>(event_queue_memory_limit_)) {
      event_msg_queue_.push(msg);
      event_queue_memory_ += msg->events.size();
    } else {
      RCLCPP_WARN(logger_, "event message queue full, dropping incoming event message!");
    }
    processEventMessages();
  }

private:
  void addNewFrame(const FrameTime & ft)
  {
    if (frames_.size() >= 100) {
      RCLCPP_WARN(logger_, "frames dropped due to frame queue overflow!");
    } else {
      frames_.push_back(ft);
    }
  }

  bool handleFirstMessage(const EventPacket::ConstSharedPtr & msg)
  {
    if (!decoder_) {
      decoder_ = decoder_factory_.getInstance(*msg);
    }
    // create temporary decoder to find the correspondence between
    // ros time and sensor time
    auto tmp_decoder = decoder_factory_.newInstance(*msg);
    uint64_t sensor_time{0};
    if (tmp_decoder->findFirstSensorTime(*msg, &sensor_time)) {
      RCLCPP_INFO_STREAM(
        logger_, "initializing: ROS time " << rclcpp::Time(msg->header.stamp).nanoseconds()
                                           << " corresponds to sensor time: " << sensor_time);
      updateRosToSensorTimeOffset(
        rclcpp::Time(msg->header.stamp), static_cast<int64_t>(sensor_time));
      // get the frame generation going if requested
      if (sensor_period_ > 0 || ros_period_ != rclcpp::Duration(0, 0)) {
        const int64_t dt = sensor_period_ > 0 ? sensor_period_ : ros_period_.nanoseconds();
        const uint64_t f_sensor = sensor_time + dt;
        addNewFrame(FrameTime(sensorToRosTime(f_sensor), f_sensor));
      } else {
        RCLCPP_ERROR(logger_, "no period set!");
        throw std::runtime_error("no period set for time slicer!");
      }
    }
    return (t0_ != std::numeric_limits<int64_t>::lowest());
  }

  void generateNextFrameIfFreeRunning()
  {
    assert(!frames_.empty());
    if (sensor_period_ > 0) {
      const uint64_t t = frames_.back().sensor_time + sensor_period_;
      addNewFrame(FrameTime(sensorToRosTime(t), t));
    } else if (ros_period_ != rclcpp::Duration(0, 0)) {
      const auto t_ros = frames_.back().ros_time + ros_period_;
      addNewFrame(FrameTime(t_ros, rosToSensorTime(t_ros)));
    }
  }

  inline uint64_t rosToSensorTime(const rclcpp::Time & t) const { return (t.nanoseconds() - t0_); }
  inline rclcpp::Time sensorToRosTime(uint64_t t) const
  {
    return (rclcpp::Time(t + t0_, RCL_ROS_TIME));
  }
  inline void updateFirstSensorTime(uint64_t sensor_time)
  {
    if (is_first_time_in_packet_) {
      updateRosToSensorTimeOffset(ros_header_time_, static_cast<int64_t>(sensor_time));
      is_first_time_in_packet_ = false;
    }
  }

  void updateRosToSensorTimeOffset(const rclcpp::Time & t_ros, int64_t t_sens)
  {
    if (t0_ == std::numeric_limits<int64_t>::lowest()) {
      t0_ = static_cast<int64_t>(t_ros.nanoseconds()) - t_sens;
      t0_init_ = t0_;
    } else {
      // to avoid rounding errors, first subtract off the large t0_init_,
      // which contains the time since epoch.
      const double dt_meas = static_cast<double>(
        static_cast<int64_t>(t_ros.nanoseconds()) - t0_init_ - t_sens);  // measured
      const double dt_k = static_cast<double>(t0_ - t0_init_);           // current estimate
      constexpr double alpha = 1.0 / 100.0;
      t0_ = t0_init_ + static_cast<int64_t>(dt_k * (1 - alpha) + dt_meas * alpha);
    }
  }

  void processEventMessages()
  {
    if (t0_ == std::numeric_limits<int64_t>::lowest()) {
      return;
    }
    auto & frames = frames_;
    while (!event_msg_queue_.empty() && !frames.empty()) {
      const auto & msg = event_msg_queue_.front();
      ros_header_time_ = rclcpp::Time(msg->header.stamp);
      while (!frames.empty()) {
        const uint64_t time_limit = frames.front().sensor_time;
        uint64_t next_time = 0;
        if (!decoder_->decodeUntil(*msg, this, time_limit, &next_time)) {
          // event message was completely decoded. Cannot emit frame yet
          // because more events may arrive that are before the frame time
          event_queue_memory_ -= msg->events.size();
          event_msg_queue_.pop();
          is_first_time_in_packet_ = true;  // for next event message
          break;
        }
        while (!frames.empty() && frames.front().sensor_time <= next_time) {
          processor_->timeSliceComplete(frames.front().sensor_time, frames.front().ros_time);
          generateNextFrameIfFreeRunning();
          frames.pop_front();
        }
      }
    }
  }
  // ------------------------  variables ------------------------------
  Processor * processor_{nullptr};
  std::string encoding_;                             // currently used incoming message encoding
  size_t event_queue_memory_{0};                     // currently used event queue memory
  int event_queue_memory_limit_{100 * 1024 * 1024};  // max event queue memory
  std::queue<EventPacket::ConstSharedPtr> event_msg_queue_;
  std::deque<FrameTime> frames_;
  int64_t t0_{std::numeric_limits<int64_t>::lowest()};  // ros to sensor time offset
  int64_t t0_init_{0};
  event_camera_codecs::Decoder<EventPacket, TimeSlicer> * decoder_{nullptr};
  event_camera_codecs::DecoderFactory<EventPacket, TimeSlicer> decoder_factory_;
  rclcpp::Time ros_header_time_;
  bool is_first_time_in_packet_{true};
  int64_t sensor_period_{0};           // sensor time period between frames
  rclcpp::Duration ros_period_{0, 0};  // ros time period between frames
  rclcpp::Logger logger_ = rclcpp::get_logger("time_slicer");
};
}  // namespace event_camera_tools
#endif  // EVENT_CAMERA_TOOLS__TIME_SLICER_H_
