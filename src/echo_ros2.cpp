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

#include <event_array_msgs/decode.h>
#include <inttypes.h>
#include <unistd.h>

#include <chrono>
#include <event_array_msgs/msg/event_array.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "echo <ros_topic>" << std::endl;
}

using event_array_msgs::msg::EventArray;
class Echo : public rclcpp::Node
{
public:
  explicit Echo(const std::string & topic, const rclcpp::NodeOptions & options)
  : Node("echo", options)
  {
    const int qsize = 1000;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qsize)).best_effort().durability_volatile();
    RCLCPP_INFO_STREAM(this->get_logger(), "subscribing to " << topic);
    sub_ = this->create_subscription<EventArray>(
      topic, qos, std::bind(&Echo::eventMsg, this, std::placeholders::_1));
  }
  void eventMsg(EventArray::ConstSharedPtr msg)
  {
    printf("--------------------------\n");
    printf("resolution: %4d  height: %4d\n", msg->width, msg->height);
    printf("encoding: %s\n", msg->encoding.c_str());
    printf("time base: %8" PRIu64 "\n", msg->time_base);
    printf("seqno: %8" PRIu64 "\n", msg->seq);
    if (msg->encoding == "mono") {
      const size_t bytes_per_event = event_array_msgs::mono::bytes_per_event;
      for (const uint8_t * p = &msg->events[0]; p != &(msg->events[0]) + msg->events.size();
           p += bytes_per_event) {
        uint16_t ex;
        uint16_t ey;
        uint64_t etu;
        bool pol = event_array_msgs::mono::decode_t_x_y_p(p, msg->time_base, &etu, &ex, &ey);
        printf("%8" PRIu64 " %4d %4d %1d\n", etu, ex, ey, pol);
      }
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "unsupported encoding: " << msg->encoding);
      throw std::runtime_error("unsupported encoding");
    }
  }
  // ---------- variables
  rclcpp::Subscription<EventArray>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  std::string topic;
  rclcpp::init(argc, argv);
  switch (argc) {
    case 2:
      topic = argv[1];
      break;
    default:
      usage();
      exit(-1);
  }
  auto node = std::make_shared<Echo>(topic, rclcpp::NodeOptions());
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return 0;
}
