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

#include <assert.h>
#include <event_array_msgs/decode.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>

#undef NDEBUG

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

class Decoder
{
public:
  typedef uint64_t timestamp_t;
  static constexpr timestamp_t MAX_BASE = (timestamp_t(1) << 12) - 1;
  static constexpr timestamp_t TIME_LOOP = 1 << 24;
  static constexpr timestamp_t LOOP_THRESH = 10;

  void processBuffer(const uint8_t * buf, size_t bufSize, EventProcessor * processor)
  {
    std::cout << "process buffer... " << std::endl;
    const size_t numRead = bufSize / sizeof(Event);
    const Event * buffer = reinterpret_cast<const Event *>(buf);
    for (size_t i = 0; i < numRead; i++) {
      switch (buffer[i].code) {
        case Code::ADDR_X: {
          const AddrX * e = reinterpret_cast<const AddrX *>(&buffer[i]);
          processor->eventCD(time_, e->x, ey_, e->polarity);
          assert(time_ > lastTime_);
          lastTime_ = time_;
          numEvents_++;
        } break;
        case Code::ADDR_Y: {
          const AddrY * e = reinterpret_cast<const AddrY *>(&buffer[i]);
          ey_ = e->y;  // save for later
        } break;
        case Code::TIME_LOW: {
          const TimeLow * e = reinterpret_cast<const TimeLow *>(&buffer[i]);
          time_ = timeBase_ | e->t;
        } break;
        case Code::TIME_HIGH: {
          const TimeHigh * e = reinterpret_cast<const TimeHigh *>(&buffer[i]);
          // find 12 lower bits of current time base
          const timestamp_t currBase = (timeBase_ >> 12) & ((1 << 12) - 1);

          // some voodoo taken from the metavision cvs converter:
          // only roll over if the new timebase is sufficiently bigger
          // than the old one. Supposedly helps against transmission errors.
          if ((currBase > e->t) && (currBase - e->t) >= (MAX_BASE - LOOP_THRESH)) {
            timeBaseRolloverBits_ += TIME_LOOP;
            std::cout << "rolling over!" << std::endl;
          }
          std::cout << "got TIME_HIGH: " << e->t << std::endl;
          // shift time base to make space for low time
          timeBase_ = (e->t << 12) | timeBaseRolloverBits_;
          time_ = timeBase_;  // will clear out current low time
        } break;
        case Code::VECT_BASE_X: {
          const VectBaseX * b = reinterpret_cast<const VectBaseX *>(&buffer[i]);
          currentPolarity_ = b->pol;
          currentBaseX_ = b->x;
          break;
        }
        case Code::VECT_8: {
          const Vect8 * b = reinterpret_cast<const Vect8 *>(&buffer[i]);
          for (int i = 0; i < 8; i++) {
            if (b->valid & (1 << i)) {
              processor->eventCD(time_, currentBaseX_ + i, ey_, currentPolarity_);
              numEvents_++;
            }
          }
          assert(time_ > lastTime_);
          lastTime_ = time_;

          currentBaseX_ += 8;
          break;
        }
        case Code::VECT_12: {
          const Vect12 * b = reinterpret_cast<const Vect12 *>(&buffer[i]);
          for (int i = 0; i < 12; i++) {
            if (b->valid & (1 << i)) {
              processor->eventCD(time_, currentBaseX_ + i, ey_, currentPolarity_);
              numEvents_++;
            }
          }
          assert(time_ > lastTime_);
          lastTime_ = time_;
          currentBaseX_ += 12;
          break;
        }
        case Code::EXT_TRIGGER: {
          const ExtTrigger * e = reinterpret_cast<const ExtTrigger *>(&buffer[i]);
          processor->eventExtTrigger(time_, e->edge, e->id);
          assert(time_ > lastTime_);
          lastTime_ = time_;
          break;
        }
        case Code::OTHERS: {
#if 0
          const Others * e = reinterpret_cast<const Others *>(&buffer[i]);
          const SubType subtype = static_cast<SubType>(e->subtype);
          if (subtype != SubType::MASTER_END_OF_FRAME) {
            std::cout << "ignoring OTHERS code: " << toString(subtype) << std::endl;
          }
#endif
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
  }
  size_t getNumEvents() const { return (numEvents_); }

private:
  // --------------------- variables
  size_t numEvents_{0};
  uint16_t ey_{0};                       // current y coordinate
  timestamp_t time_{0};                  // complete accumulated sensor timestamp
  timestamp_t timeBase_{0};              // time stamp base (time_high), shifted by 12 bits left
  timestamp_t timeBaseRolloverBits_{0};  // bits from rollover, shifted by 24 bits left
  timestamp_t lastTime_{0};              // last decoded time
  uint8_t currentPolarity_{0};           // polarity for vector event
  uint16_t currentBaseX_{0};             // X coordinate basis for vector event
  timestamp_t lastStamp_{0};
};

size_t read(const std::string & inFile, EventProcessor * processor)
{
  std::fstream in;
  in.open(inFile, std::ios::in | std::ios::binary);
  skip_header(in);
  char buffer[1000 * sizeof(Event)];
  Decoder decoder;
  while (in.read(buffer, sizeof(buffer))) {
    decoder.processBuffer(reinterpret_cast<uint8_t *>(buffer), in.gcount(), processor);
    processor->rawData(reinterpret_cast<char *>(buffer), in.gcount());
  }
  processor->finished();
  return (decoder.getNumEvents());
}

}  // namespace evt_3_utils
}  // namespace event_array_tools
