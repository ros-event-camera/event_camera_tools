// -*-c++-*----------------------------------------------------------------------------------------
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

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>
#include <inttypes.h>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <list>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <unordered_map>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "trigger_delay -t <trigger_image_topic> [-d (use down edge)] <event_topic>"
            << std::endl;
}

static rclcpp::Logger logger() { return (rclcpp::get_logger("trigger_delay")); }

namespace event_camera_tools
{
using TriggerType = sensor_msgs::msg::Image;
using EventPacket = event_camera_msgs::msg::EventPacket;
using EventPacketConstPtr = EventPacket::ConstSharedPtr;

class TriggerDelay;  // forward decl
class EventSub : public event_camera_codecs::EventProcessor
{
public:
  EventSub() = default;
  // ---------- from the EventProcessor interface:
  void eventCD(uint64_t sensor_time, uint16_t, uint16_t, uint8_t) override
  {
    if (is_first_event_in_packet_) {
      is_first_event_in_packet_ = false;
      first_sensor_time_ = sensor_time;
    }
  }
  bool eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t id) override;
  void finished() override {}
  void rawData(const char *, size_t) override {}
  // ------------ end inherited methods
  void setEdge(uint8_t edge) { edge_ = edge; }
  void eventCallback(const EventPacketConstPtr & msg);
  // ----- variables --------
  rclcpp::Subscription<EventPacket>::SharedPtr sub_;
  event_camera_codecs::DecoderFactory<EventPacket, EventSub> decoder_factory_;
  bool is_first_event_in_packet_{true};
  uint64_t first_sensor_time_{0};
  uint64_t header_stamp_{0};
  TriggerDelay * trigger_delay_{0};
  uint8_t edge_{1};  // rising edge
};

class TriggerDelay : public rclcpp::Node
{
public:
  explicit TriggerDelay(
    const std::string & event_topic, const std::string & trigger_topic, uint8_t edge,
    const rclcpp::NodeOptions & options)
  : Node("trigger_delay", options)
  {
    event_sub_.trigger_delay_ = this;  // give back pointer
    event_sub_.setEdge(edge);
    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(1.0),
      [=]() { this->statsTimerExpired(); });
    RCLCPP_INFO_STREAM(logger(), "subscribing to events on " << event_topic);
    RCLCPP_INFO_STREAM(logger(), "subscribing for trigger frames on " << trigger_topic);
    RCLCPP_INFO_STREAM(logger(), "using " << (edge ? "UP" : "DOWN") << " edge trigger");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    event_sub_.sub_ = this->create_subscription<EventPacket>(
      event_topic, qos, std::bind(&EventSub::eventCallback, &event_sub_, std::placeholders::_1));
    trigger_sub_ = this->create_subscription<TriggerType>(
      trigger_topic, qos, std::bind(&TriggerDelay::frameCallback, this, std::placeholders::_1));
  }

  static void updateCount(
    uint64_t t, uint64_t * last_time, std::list<uint64_t> * times, size_t * num,
    uint64_t * total_time, double * period)
  {
    if (*last_time != 0) {
      const uint64_t dt = t - *last_time;
      *total_time += static_cast<double>(dt);
      *num = *num + 1;
      *period = (*period == 0) ? dt : (*period * 0.9 + dt * 0.1);
    }
    *last_time = t;
    while (times->size() >= 10) {
      times->pop_front();
    }
    times->push_back(t);
  }

  void triggerEvent(uint64_t t)
  {
    updateCount(
      t, &trigger_last_time_, &trigger_times_, &num_triggers_, &total_trigger_time_,
      &trigger_period_);
    checkFramesVsTriggerEvents();
  }

private:
  void frameCallback(const TriggerType::ConstSharedPtr & msg)
  {
    const uint64_t t = rclcpp::Time(msg->header.stamp).nanoseconds();
    updateCount(
      t, &frame_last_time_, &frame_times_, &num_frames_, &total_frame_time_, &frame_period_);
    checkFramesVsTriggerEvents();
  }

  void checkFramesVsTriggerEvents()
  {
    const uint64_t period_match = static_cast<uint64_t>(trigger_period_) >> 2;
    if (period_match == 0) {
      RCLCPP_WARN_THROTTLE(
        logger(), *this->get_clock(), 1000,
        "no event triggers received yet to compute trigger period!");
      return;
    }

    for (auto t_it_loop = trigger_times_.begin(); t_it_loop != trigger_times_.end();) {
      auto t_it = t_it_loop++;
      for (auto f_it_loop = frame_times_.begin(); f_it_loop != frame_times_.end();) {
        auto f_it = f_it_loop++;
        if ((*t_it + period_match > *f_it) && (*t_it < *f_it + period_match)) {
          const int64_t dt = static_cast<int64_t>(*t_it) - static_cast<int64_t>(*f_it);
          total_trigger_delay_ += dt;
          num_trigger_delays_++;
          frame_times_.erase(frame_times_.begin(), f_it_loop);  // does not include the stop iter!
          trigger_times_.erase(trigger_times_.begin(), t_it_loop);
        }
      }
    }
  }

  void statsTimerExpired()
  {
    const double frame_rate =
      total_frame_time_ > 0 ? num_frames_ / static_cast<double>(total_frame_time_) : 0;
    const double trigger_rate =
      total_trigger_time_ > 0 ? num_triggers_ / static_cast<double>(total_trigger_time_) : 0;
    const double trigger_delay =
      num_trigger_delays_ > 0 ? static_cast<double>(total_trigger_delay_) / num_trigger_delays_ : 0;
    RCLCPP_INFO(
      logger(), "frame rate: %7.3lf Hz, trigger rate: %7.3lf Hz, trigger delay: %6.3lf ms",
      frame_rate * 1e9, trigger_rate * 1e9, trigger_delay * 1e-6);
    num_triggers_ = 0;
    total_trigger_time_ = 0;
    num_frames_ = 0;
    total_frame_time_ = 0;
    total_trigger_delay_ = 0;
    num_trigger_delays_ = 0;
  }
  // ---------  variables
  rclcpp::TimerBase::SharedPtr timer_;
  EventSub event_sub_;
  rclcpp::Subscription<TriggerType>::SharedPtr trigger_sub_;
  std::list<uint64_t> frame_times_;
  std::list<uint64_t> trigger_times_;
  uint64_t total_trigger_time_{0};
  uint64_t trigger_last_time_{0};
  size_t num_triggers_{0};
  uint64_t total_frame_time_{0};
  uint64_t frame_last_time_{0};
  size_t num_frames_{0};
  int64_t total_trigger_delay_{0};
  size_t num_trigger_delays_{0};
  double trigger_period_{0};
  double frame_period_{0};
};

// must define here because of reference to TriggerDelay class
bool EventSub::eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t)
{
  if (is_first_event_in_packet_) {
    is_first_event_in_packet_ = false;
    first_sensor_time_ = sensor_time;
  }
  if (edge == edge_) {
    trigger_delay_->triggerEvent(sensor_time + header_stamp_ - first_sensor_time_);
  }
  return (true);
}

void EventSub::eventCallback(const EventPacketConstPtr & msg)
{
  header_stamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  is_first_event_in_packet_ = true;
  if (!msg->events.empty()) {
    auto decoder = decoder_factory_.getInstance(*msg);
    if (!decoder) {
      printf("unsupported encoding: %s\n", msg->encoding.c_str());
      return;
    }
    decoder->decode(*msg, this);
  }
}

}  // namespace event_camera_tools

int main(int argc, char ** argv)
{
  int opt;

  rclcpp::init(argc, argv);

  std::string event_topic;
  std::string trigger_topic;
  uint8_t edge = 1;  // default: rising edge
  while ((opt = getopt(argc, argv, "t:d")) != -1) {
    switch (opt) {
      case 'd':
        edge = 0;  // use falling edge
        break;
      case 't':
        trigger_topic = optarg;
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
    std::cout << "expected event topic!" << std::endl;
    usage();
    return (-1);
  }
  if (trigger_topic.empty()) {
    std::cout << "trigger topic must be specified!" << std::endl;
    usage();
    return (-1);
  }
  event_topic = argv[optind];
  auto node = std::make_shared<event_camera_tools::TriggerDelay>(
    event_topic, trigger_topic, edge, rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return (0);
}
