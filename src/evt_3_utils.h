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

#ifndef EVT_3_UTILS_H_
#define EVT_3_UTILS_H_

#include <fstream>
#include <string>

namespace event_array_tools
{
namespace evt_3_utils
{
class MessageUpdater
{
public:
  ~MessageUpdater() {}
  virtual void addEvent(uint64_t ts_ros, uint16_t ex, uint16_t ey, uint8_t polarity) = 0;
  virtual uint64_t getROSTime() = 0;
  virtual void finished() = 0;
  virtual void bufferRead(const char * data, size_t len) = 0;
};

enum Code {
  ADDR_Y = 0b0000,       // 0
  ADDR_X = 0b0010,       // 2
  VECT_BASE_X = 0b0011,  // 3
  VECT_12 = 0b0100,      // 4
  VECT_8 = 0b0101,       // 5
  TIME_LOW = 0b0110,     // 6
  CONTINUED_4 = 0b0111,  // 7
  TIME_HIGH = 0b1000,    // 8
  EXT_TRIGGER = 0b1010,  // 10
  OTHERS = 0b1110,       // 14
  CONTINUED_12 = 0b1111  // 15
};

enum SubType {
  MASTER_SYSTEM_TEMPERATURE = 0x000,
  MASTER_SYSTEM_VOLTAGE = 0x001,
  MASTER_SYSTEM_IN_EVENT_COUNT = 0x002,
  MASTER_SYSTEM_IN_EVENT_SEQ_ERROR = 0x003,
  MASTER_SYSTEM_IN_EVENT_TIME_ERROR = 0x004,
  MASTER_SYSTEM_OUT_EVENT_COUNT = 0x005,
  MASTER_SYSTEM_OUT_EVENT_SEQ_ERROR = 0x006,
  MASTER_SYSTEM_OUT_EVENT_TIME_ERROR = 0x007,
  MASTER_IN_TD_EVENT_COUNT = 0x014,
  MASTER_IN_APS_EVENT_COUNT = 0x015,
  MASTER_RATE_CONTROL_TD_EVENT_COUNT = 0x016,
  MASTER_RATE_CONTROL_APS_EVENT_COUNT = 0x017,
  MASTER_START_OF_FRAME = 0x018,
  MASTER_END_OF_FRAME = 0x019,
  MASTER_MIPI_PADDING = 0x01A,
  LOW_POWER_STATE_ENTRY_1 = 0x020,
  LOW_POWER_STATE_ENTRY_2 = 0x021,
  LOW_POWER_STATE_DEEP_EXIT = 0x022,
  END_OF_TEST_TASK = 0x0FD,
  USB_PACKET_INFO = 0x0FE,
  DUMMY = 0x0FF,
  MASTER_TL_DROP_EVENT = 0xED6,
  MASTER_TH_DROP_EVENT = 0xED8,
  MASTER_EVT_DROP_EVENT = 0xEDA
};

struct __attribute__((packed)) Event
{
  unsigned int rest : 12;
  unsigned int code : 4;
};

struct __attribute__((packed)) AddrY
{
  AddrY(uint16_t y_a, uint8_t s) : y(y_a), system_type(s), code(Code::ADDR_Y) {}
  void write(std::fstream & out) const
  {
    out.write(reinterpret_cast<const char *>(this), sizeof(*this));
  }
  unsigned int y : 11;
  unsigned int system_type : 1;
  unsigned int code : 4;
};

struct __attribute__((packed)) AddrX
{
  AddrX(uint16_t x_a, uint8_t p) : x(x_a), polarity(p), code(Code::ADDR_X) {}
  void write(std::fstream & out) const
  {
    out.write(reinterpret_cast<const char *>(this), sizeof(*this));
  }
  unsigned int x : 11;
  unsigned int polarity : 1;
  unsigned int code : 4;
};

struct __attribute__((packed)) TimeHigh
{
  TimeHigh(uint32_t ts_usec) : t(ts_usec >> 12), code(Code::TIME_HIGH) {}
  uint32_t write(std::fstream & out) const
  {
    out.write(reinterpret_cast<const char *>(this), sizeof(*this));
    return (static_cast<uint32_t>(t) << 12);
  }
  unsigned int t : 12;
  unsigned int code : 4;
};

struct __attribute__((packed)) TimeLow
{
  TimeLow(uint32_t ts_usec) : t(ts_usec & 0x00000FFF), code(Code::TIME_LOW) {}
  void write(std::fstream & out) const
  {
    out.write(reinterpret_cast<const char *>(this), sizeof(*this));
  }
  unsigned int t : 12;
  unsigned int code : 4;
};

struct __attribute__((packed)) Others
{
  Others(uint16_t s) : subtype(s), code(Code::OTHERS) {}
  void write(std::fstream & out) const
  {
    out.write(reinterpret_cast<const char *>(this), sizeof(*this));
  }
  unsigned int subtype : 12;
  unsigned int code : 4;
};

std::string toString(const SubType s);

size_t write(
  std::fstream & out, const uint8_t * p, const size_t num_bytes, const uint64_t time_base,
  const std::string & encoding, uint32_t * last_evt_stamp);

size_t read(const std::string & inFile, MessageUpdater * updater);
}  // namespace evt_3_utils
}  // namespace event_array_tools

#endif  // EVT_3_UTILS_H_
