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

#include "evt_3_utils.h"

#include <event_array_msgs/decode.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>

namespace event_array_tools
{
using event_array_msgs::mono::bytes_per_event;
using event_array_msgs::mono::decode_t_x_y_p;

namespace evt_3_utils
{
std::string toString(const SubType s)
{
  switch (s) {
    case MASTER_SYSTEM_TEMPERATURE:
      return ("MASTER_SYSTEM_TEMPERATURE");
    case MASTER_SYSTEM_VOLTAGE:
      return ("MASTER_SYSTEM_VOLTAGE");
    case MASTER_SYSTEM_IN_EVENT_COUNT:
      return ("MASTER_SYSTEM_IN_EVENT_COUNT");
    case MASTER_SYSTEM_IN_EVENT_SEQ_ERROR:
      return ("MASTER_SYSTEM_IN_EVENT_SEQ_ERROR");
    case MASTER_SYSTEM_IN_EVENT_TIME_ERROR:
      return ("MASTER_SYSTEM_IN_EVENT_TIME_ERROR");
    case MASTER_SYSTEM_OUT_EVENT_COUNT:
      return ("MASTER_SYSTEM_OUT_EVENT_COUNT");
    case MASTER_SYSTEM_OUT_EVENT_SEQ_ERROR:
      return ("MASTER_SYSTEM_OUT_EVENT_SEQ_ERROR");
    case MASTER_SYSTEM_OUT_EVENT_TIME_ERROR:
      return ("MASTER_SYSTEM_OUT_EVENT_TIME_ERROR");
    case MASTER_IN_TD_EVENT_COUNT:
      return ("MASTER_IN_TD_EVENT_COUNT");
    case MASTER_IN_APS_EVENT_COUNT:
      return ("MASTER_IN_APS_EVENT_COUNT");
    case MASTER_RATE_CONTROL_TD_EVENT_COUNT:
      return ("MASTER_RATE_CONTROL_TD_EVENT_COUNT");
    case MASTER_RATE_CONTROL_APS_EVENT_COUNT:
      return ("MASTER_RATE_CONTROL_APS_EVENT_COUNT");
    case MASTER_START_OF_FRAME:
      return ("MASTER_START_OF_FRAME");
    case MASTER_END_OF_FRAME:
      return ("MASTER_END_OF_FRAME");
    case MASTER_MIPI_PADDING:
      return ("MASTER_MIPI_PADDING");
    case LOW_POWER_STATE_ENTRY_1:
      return ("LOW_POWER_STATE_ENTRY_1");
    case LOW_POWER_STATE_ENTRY_2:
      return ("LOW_POWER_STATE_ENTRY_2");
    case LOW_POWER_STATE_DEEP_EXIT:
      return ("LOW_POWER_STATE_DEEP_EXIT");
    case END_OF_TEST_TASK:
      return ("END_OF_TEST_TASK");
    case USB_PACKET_INFO:
      return ("USB_PACKET_INFO");
    case DUMMY:
      return ("DUMMY");
    case MASTER_TL_DROP_EVENT:
      return ("MASTER_TL_DROP_EVENT");
    case MASTER_TH_DROP_EVENT:
      return ("MASTER_TH_DROP_EVENT");
    case MASTER_EVT_DROP_EVENT:
      return ("MASTER_EVT_DROP_EVENT");
  }
  return ("UKNOWN");
}

size_t write(
  std::fstream & out, const uint8_t * p, const size_t num_bytes, const uint64_t time_base,
  const std::string & encoding, uint32_t * last_evt_stamp)
{
  if (encoding != "mono") {
    std::cerr << "unknown event encoding: " << encoding << std::endl;
    throw std::runtime_error("unknown encoding!");
  }

  uint32_t last_etu_us(0);
  for (size_t i = 0; i < num_bytes; i += bytes_per_event, p += bytes_per_event) {
    uint16_t ex;
    uint16_t ey;
    uint64_t etu;
    bool pol = decode_t_x_y_p(p, time_base, &etu, &ex, &ey);
    const uint64_t sensor_us = etu / 1000;  // total sensor time in usec
    const uint32_t etu_us = sensor_us & 0x0000000000FFFFFF;
    if (*last_evt_stamp == 0 || (*last_evt_stamp != (etu_us & 0xFFFFF000))) {
      TimeHigh e3th(etu_us);
      *last_evt_stamp = e3th.write(out);
    }
    // write last digits of time event if first event in ROS message,
    // or if the event time stamp has changed,
    if (i == 0 || last_etu_us != etu_us) {
      TimeLow e3tl(etu_us);
      e3tl.write(out);
      last_etu_us = etu_us;
    }
    AddrY e3y(ey, 0);
    e3y.write(out);
    AddrX e3x(ex, static_cast<uint8_t>(pol));
    e3x.write(out);
  }
  return (num_bytes / bytes_per_event);
}

static void skip_header(std::fstream & in)
{
  int c;
  while ((c = in.peek()) == '%') {
    std::string line;
    std::getline(in, line);
  }
}

size_t read(const std::string & inFile, MessageUpdater * updater)
{
  std::fstream in;
  in.open(inFile, std::ios::in | std::ios::binary);
  skip_header(in);
  const size_t BUF_SIZE(100);
  Event buffer[BUF_SIZE];

  size_t numEvents(0);
  uint16_t ey = 0;
  uint64_t ts(0);  // sensor time stamp accumulated from LOW and HIGH messages
  uint64_t t0_sensor(0);

  while (in.read(reinterpret_cast<char *>(buffer), BUF_SIZE * sizeof(Event))) {
    size_t numRead = in.gcount() / sizeof(Event);
    for (size_t i = 0; i < numRead; i++) {
      switch (buffer[i].code) {
        case Code::ADDR_X: {
          const AddrX * e = reinterpret_cast<const AddrX *>(&buffer[i]);
          if (t0_sensor == 0) {  // this is the first full event ever received
            t0_sensor = ts;
          }
          // ROS time in nanoseconds
          const uint64_t ts_ros = updater->getROSTime() + (ts - t0_sensor) * 1000LL;
          updater->addEvent(ts_ros, e->x, ey, e->polarity);
          numEvents++;
        } break;
        case Code::ADDR_Y: {
          const AddrY * e = reinterpret_cast<const AddrY *>(&buffer[i]);
          ey = e->y;  // save for later
        } break;
        case Code::TIME_LOW: {
          const TimeLow * e = reinterpret_cast<const TimeLow *>(&buffer[i]);
          ts = (ts & 0xFFFFFFFFFFFFF000) | (e->t & 0xFFF);
        } break;
        case Code::TIME_HIGH: {
          const TimeHigh * e = reinterpret_cast<const TimeHigh *>(&buffer[i]);
          const uint64_t prev_ts = ts;
          ts = (e->t << 12);
          if (ts < (prev_ts & 0xFFFFFFFFFFFFF000)) {
            // must have experienced wrap-around
            t0_sensor = t0_sensor - (prev_ts - ts);
          }
        } break;
        case Code::OTHERS: {
          const Others * e = reinterpret_cast<const Others *>(&buffer[i]);
          const SubType subtype = static_cast<SubType>(e->subtype);
          if (subtype != SubType::MASTER_END_OF_FRAME) {
            std::cout << "ignoring OTHERS code: " << toString(subtype) << std::endl;
          }
        } break;
          // ------- the CONTINUED codes are used in conjunction with
          // the OTHERS code, so ignore as well
        case Code::CONTINUED_4:
        case Code::CONTINUED_12: {
        } break;
        default:
          // ------- all the vector codes are not generated
          // by the Gen3 sensor I have....
          std::cout << "got unsupported code: " << static_cast<int>(buffer[i].code) << std::endl;
          throw std::runtime_error("got unsupported code!");
          break;
      }
    }
    updater->bufferRead(reinterpret_cast<char *>(buffer), in.gcount());
  }
  updater->finished();
  return (numEvents);
}

}  // namespace evt_3_utils
}  // namespace event_array_tools
