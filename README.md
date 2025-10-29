# event_camera_tools

This repository holds ROS2 tools for displaying and converting
[event_camera_msgs](https://github.com/ros-event-camera/event_camera_msgs)
under ROS2. These messages are produced by the
[metavision_driver](https://github.com/ros-event-camera/metavision_driver) and
the [libcaer_driver](https://github.com/ros-event-camera/libcaer_driver).

## Supported platforms

NOTE: ROS1 support has been discontinued.
ROS2 is supported for ROS2 Humble and later distros.

## How to build

Set the following shell variables:
```bash
repo=event_camera_tools
url=https://github.com/ros-event-camera/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## Republish conversion nodelet

The ``republish`` node converts event_camera_msgs to legacy formats
like dvs, prophesee, and decoded ("mono") event_camera messages. Note
that this node will consume a significant amount of CPU
resources and should not be run unnecessarily while recording data.
The following command will start a conversion node to republish
events from ``/event_camera/events`` to
``/event_camera/republished_events`` and
``/event_camera/republished_triggers``
(see launch file for remapping):

```bash
ros2 launch event_camera_tools republish_composable.launch.py camera:=event_camera message_type:=event_packet
```

## Tools

- ``ros2 run event_camera_tools echo [-b <bag>] [-n (no header printed)] <topic>``  
  Displays messages in ``event_camera_msgs`` format, optionally from a bag file. Example output:

```text
  -------------------------------
  res:  640  height:  480 enc: evt3
  header stamp: 1664227781775114816
  time base:        0
  seqno:   129213
  ---
  6851488000 edge: 1  id:  6
  6851510000  390  223 1
  6851520000   33  326 0
  ...
```

- ``ros2 run event_camera_tools perf <topic>``  
  Sample output:

  ```text
  msgs:   219.48/s drp:  0 del: 13.72ms drft: 0.0033s ev:   0.0823 M/s %ON:  46 tr:  1758.38 1/s %UP:  50
  msgs:   249.01/s drp:  0 del:  4.35ms drft: 0.0027s ev:   0.8497 M/s %ON:  52 tr:  1999.56 1/s %UP:  49
   ```

  The meaning of the fields is as follows:
  - ``msgs`` message rate per seconds
  - ``drp`` number of drops per second (skip in sequence numbers)
  - ``del`` delay: average time difference between message header
     stamp and arrival time. This includes the delay due to the driver
     aggregating messages.
  - ``drft`` accumulated (from start of "perf") drift between message
     header stamp and sensor-provided time.
  - ``ev`` event rate in millions/sec
  - ``%ON`` ratio of ON events to total (ON + OFF) events
  - ``tr`` rate of trigger messages
  - ``%UP`` ratio of UP trigger edges to total (UP + DOWN)
- ``ros2 run event_camera_tools sync_test <cam_0_event_topic> <cam_1_event_topic>``
  The output gives the average sensor time difference and how many
  samples where counted:

  ```text
  avg sensor diff:  0.00846s, count:   360
  avg sensor diff:  0.00377s, count:   466
  ...
  ```

- ``ros2 run event_camera_tools bag_to_raw -t <topic> -b <bag_name> -o <outout_raw_file> -c <camera_type>``

  Converts bags with evt3 event_camera_msgs to raw file. The ``camera_type`` argument is necessary to produce a valid header for
  the raw file.
- ``ros2 run event_camera_tools raw_to_bag -t <topic> -b <bag_file> -i <input_raw_file> -w <sensor_width> -h <sensor_height> -p <packet_duration_ms>``

  Converts raw file into bag with evt3 event_camera_msgs. The ``packet_duration_ms`` gives the time slice (in  milliseconds) per ROS packet. The default is 10ms. Choose this parameter smaller to get lower processing latencies.

- ``ros2 run event_camera_tools movie_maker -f <fps> -b <bag_name> -t <topic>``

  Produces sequence of frame images.

- ``ros2 run event_camera_tools legacy_to_bag -b <input_bag_file> -o <output_bag_file> -t <topic1> [topic2 topic3 ...]``

  Converts bags with DVS or Prophesee messages to evt3 event_camera_msgs.

- ``ros2 run event_camera_tools find_trigger_events -i input_bag -t <topic_with_trigger_events>``

  This tool is useful when aligning reconstructed image frames for datasets that use a pulse-per-second synchronization scheme. The ``find_trigger_events`` tool finds the ROS time and sensor time of the first trigger pulse in a rosbag. Output example:

  ```text
  first trigger ROS    time: 1702577874035860000
  first trigger sensor time: 24809136000
  num triggers: 1712
  avg time between triggers:
  ROS time:    0.0499567s
  sensor time: 0.0499567s
  processed 202 number of messages
  ```

  The ROS and sensor times of the first trigger event serve as inputs for [image reconstruction tools](https://github.com/berndpfrommer/simple_image_recon) such as ``bag_to_frames``.

- ``ros2 run event_camera_tools plot_events -b name_of_bag -o name_of_plot_file -t <topic> [-z]``

  Creates file with 5 columns (sensor time, ros time, x, y, polarity) for plotting. Also useful for clear text viewing of events in bag file.

- ``ros2 run event_camera_tools event_statics -b name_of_bag -t <topic> [-s <scale_file>]``

  Computes the per-pixel number of ON and OFF events in a bag and writes them to ``scale_file.txt`` (default).
  The file has alternatingly the number of OFF and ON events in row major order, with line breaks after each row.

## License

This software is issued under the Apache License Version 2.0.
