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

#include <event_array_msgs/EventArray.h>
#include <event_array_msgs/decode.h>
#include <ros/ros.h>
#include <unistd.h>

#include <chrono>
#include <fstream>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "perf <ros_topic>" << std::endl;
}

using event_array_msgs::EventArray;

class Perf
{
public:
  explicit Perf(const ros::NodeHandle & nh, const std::string & topic) : nh_(nh)
  {
    const int qsize = 1000;
    ROS_INFO_STREAM("subscribing to " << topic);
    sub_ = nh_.subscribe(topic, qsize, &Perf::eventMsg, this);
    timer_ = nh_.createTimer(ros::Duration(2.0), &Perf::timerExpired, this);
    lastTime_ = ros::Time::now();
  }
  void eventMsg(const EventArray::ConstPtr & msg)
  {
    size_t idx = 0;
    size_t bytes_per_event = 0;
    if (msg->encoding == "mono") {
      idx = 0;
      bytes_per_event = event_array_msgs::mono::bytes_per_event;
    } else if (msg->encoding == "trigger") {
      idx = 1;
      bytes_per_event = event_array_msgs::trigger::bytes_per_event;
    } else {
      ROS_ERROR_STREAM("unsupported encoding: " << msg->encoding);
      throw std::runtime_error("unsupported encoding");
    }
    const ros::Time t_stamp = ros::Time(msg->header.stamp);
    if (lastSeq_[idx] == 0) {
      lastSeq_[idx] = msg->seq - 1;
      lastStamp_ = t_stamp;
    }
    numMsgs_[idx]++;
    const int num_dropped = msg->seq - 1 - lastSeq_[idx];
    dropped_[idx] += num_dropped;
    lastSeq_[idx] = msg->seq;
    if (t_stamp < lastStamp_) {
      numReverse_[idx]++;
    }
    lastStamp_ = t_stamp;
    numEvents_[idx] += msg->events.size() / bytes_per_event;
    const auto t = ros::Time::now();
    delay_[idx] += (t - t_stamp).toNSec();
  }

  void timerExpired(const ros::TimerEvent &)
  {
    const auto t = ros::Time::now();
    const double dt = (t - lastTime_).toNSec() * 1e-9;  // in milliseconds
    for (int i = 0; i < 2; i++) {
      if (numEvents_[i] != 0 && numMsgs_[i] != 0) {
        const std::string label = i == 0 ? "events" : "triggr";
        printf(
          "%s: %8.4f M/s msgs: %8.2f/s drop: %3zu delay: %5.2fms", label.c_str(),
          numEvents_[i] / dt * 1e-6, numMsgs_[i] / dt, dropped_[i],
          delay_[i] / (1e6 * numMsgs_[i]));
        if (numReverse_[i] != 0) {
          printf(" ROS STAMP REVERSED: %3zu\n", numReverse_[i]);
        } else {
          printf("\n");
        }
      }
      numEvents_[i] = numMsgs_[i] = delay_[i] = dropped_[i] = numReverse_[i] = 0;
    }
    lastTime_ = t;
  }
  // ------------ variables
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Timer timer_;
  size_t numMsgs_[2]{0, 0};
  size_t numEvents_[2]{0, 0};
  int64_t delay_[2]{0, 0};
  size_t dropped_[2]{0, 0};
  size_t numReverse_[2]{0, 0};
  uint64_t lastSeq_[2]{0, 0};
  ros::Time lastTime_;
  ros::Time lastStamp_;
};

int main(int argc, char ** argv)
{
  std::string topic;
  ros::init(argc, argv, "driver_node");

  switch (argc) {
    case 2:
      topic = argv[1];
      break;
    default:
      usage();
      exit(-1);
  }

  ros::NodeHandle pnh("~");
  Perf perf(pnh, topic);
  ros::spin();
  return 0;
}
