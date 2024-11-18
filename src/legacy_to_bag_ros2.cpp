#include <event_camera_codecs/encoder.h>
#include <unistd.h>

#include <chrono>
#include <dvs_msgs/msg/event_array.hpp>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <iostream>
#include <prophesee_event_msgs/msg/event_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <vector>

#include "event_camera_tools/check_endian.h"

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "legacy_to_bag -b name_of_legacy_bag_file -o name_of_new_bag -t topic " << std::endl;
}

namespace event_camera_tools
{

static uint64_t seq{0};
template <typename MsgType>
static auto processMsg(
  const typename MsgType::SharedPtr & inMsg, const std::string & topic,
  rosbag2_cpp::Writer * outBag, event_camera_codecs::Encoder * enc) -> size_t
{
  if (inMsg->events.empty()) {
    return (0);  // drop empty messages
  }
  event_camera_msgs::msg::EventPacket outMsg;
  outMsg.header = inMsg->header;
  enc->setBuffer(&outMsg.events);
  // use first event time to set sensor time
  const rclcpp::Time firstEventTime(inMsg->events[0].ts);
  outMsg.time_base = firstEventTime.nanoseconds();
  outMsg.height = inMsg->height;
  outMsg.width = inMsg->width;
  outMsg.encoding = "evt3";
  outMsg.seq = seq++;
  outMsg.is_bigendian = check_endian::isBigEndian();
  enc->setSensorTime(firstEventTime.nanoseconds());
  // use encoder to convert events
  for (const auto & e : inMsg->events) {
    enc->encodeCD((e.ts.nanosec - firstEventTime.nanoseconds()), e.x, e.y, e.polarity);
  }
  enc->flush();
  outBag->write(outMsg, topic, inMsg->header.stamp);
  return (inMsg->events.size());
}

static auto process_bag(
  const std::string & inBagName, const std::string & outBagName,
  const std::vector<std::string> & topics) -> size_t
{
  std::cout << "reading from bag: " << inBagName << " topics: [";
  for (const auto & topic : topics) {
    std::cout << topic << ", ";
  }
  std::cout << "]" << std::endl;
  std::cout << "writing to bag: " << outBagName << std::endl;

  auto reader = std::make_unique<rosbag2_cpp::Reader>();
  reader->open(inBagName);

  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  writer->open(outBagName);

  auto encoder = event_camera_codecs::Encoder::newInstance("evt3");
  if (!encoder) {
    std::cerr << "evt3 codec not supported by encoder!" << std::endl;
    throw(std::runtime_error("evt3 unsupported!"));
  }

  for (const auto & topic_meta : reader->get_all_topics_and_types()) {
    if (
      topic_meta.type != "dvs_msgs/msg/EventArray" &&
      topic_meta.type != "prophesee_event_msgs/msg/EventArray") {
      writer->create_topic(topic_meta);
    } else {
      rosbag2_storage::TopicMetadata meta = topic_meta;
      meta.type = "event_camera_msgs/msg/EventPacket";
      writer->create_topic(meta);
    }
  }

  size_t numMessages(0);
  size_t numEvents(0);

  auto dvs_serializer = rclcpp::Serialization<dvs_msgs::msg::EventArray>();
  auto proph_serializer = rclcpp::Serialization<prophesee_event_msgs::msg::EventArray>();
  while (reader->has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();

    if (std::find(topics.begin(), topics.end(), msg->topic_name) != topics.end()) {
      rcutils_uint8_array_t raw_data = *msg->serialized_data;
      auto buffer = rclcpp::SerializedMessage(raw_data);
      try {
        dvs_msgs::msg::EventArray::SharedPtr dvs = std::make_shared<dvs_msgs::msg::EventArray>();
        dvs_serializer.deserialize_message(&buffer, dvs.get());
        numEvents +=
          processMsg<dvs_msgs::msg::EventArray>(dvs, msg->topic_name, writer.get(), encoder.get());
        numMessages++;
      } catch (const rclcpp::exceptions::RCLError & e) {
        try {
          prophesee_event_msgs::msg::EventArray::SharedPtr proph =
            std::make_shared<prophesee_event_msgs::msg::EventArray>();
          proph_serializer.deserialize_message(&buffer, proph.get());
          numEvents += processMsg<prophesee_event_msgs::msg::EventArray>(
            proph, msg->topic_name, writer.get(), encoder.get());
          numMessages++;
        } catch (const std::exception & e) {
          std::cerr << "Failed to deserialize message on topic " << msg->topic_name << std::endl;
          continue;
        }
      }
    } else {
      // write the message as is
      writer->write(msg);
    }
  }
  reader->close();
  writer->close();
  std::cout << "read " << numMessages << " messages" << std::endl;
  return (numEvents);
}
}  // namespace event_camera_tools

auto main(int argc, char ** argv) -> int
{
  int opt;
  rclcpp::init(argc, argv);
  std::string inBag;
  std::string outBag;
  std::vector<std::string> topics;
  while ((opt = getopt(argc, argv, "b:o:t")) != -1) {
    switch (opt) {
      case 'b':
        inBag = optarg;
        break;
      case 'o':
        outBag = optarg;
        break;
      case 't':
        while (optind < argc && argv[optind][0] != '-') {
          topics.emplace_back(argv[optind]);
          optind++;
        }
        break;
      default:
        usage();
        rclcpp::shutdown();
        return 1;
    }
  }
  if (inBag.empty() || outBag.empty()) {
    std::cout << "missing input or output bag name!" << std::endl;
    usage();
    return (-1);
  }

  const auto start = std::chrono::high_resolution_clock::now();

  const size_t numEvents = event_camera_tools::process_bag(inBag, outBag, topics);

  const auto end = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  std::cout << "total number of events: " << numEvents << std::endl;
  std::cout << "event rate: "
            << static_cast<double>(numEvents) / static_cast<double>(total_duration.count())
            << " Mev/s" << std::endl;

  return (0);
}