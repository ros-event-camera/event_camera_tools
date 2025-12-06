# event\_camera\_tools

This repository holds ROS tools for displaying and converting
[event_camera_msgs](https://github.com/ros-event-camera/event_camera_msgs).
These messages are produced by the
[metavision_driver](https://github.com/ros-event-camera/metavision_driver) and
the [libcaer_driver](https://github.com/ros-event-camera/libcaer_driver).
For converting legacy [dvs\_msgs](https://github.com/ros-event-camera/dvs_msgs) and
[prophesee\_event\_msgs](https://github.com/ros-event-camera/prophesee_event_msgs)
messages, see the [event\_camera\_legacy\_tools](https://github.com/ros-event-camera/event_camera_legacy_tools)
repository.

## Supported platforms

NOTE: ROS1 support has been discontinued but the code has been left in place and may still compile.
ROS2 is supported for ROS2 Humble and later distros.

## How to build

Set the following shell variables:

```bash
repo=event_camera_tools
url=https://github.com/ros-event-camera/${repo}.git
```

and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## Tools

### Real-time monitoring tools

- ``ros2 run event_camera_tools echo [-b <bag>] [-t (only triggers)] [-n (no header printed)] <topic>``  

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

- ``ros2 run event_camera_tools trigger_delay -t <camera_image_topic> [-d (use down trigger edge)] <event_topic>``

  Use this tool to verify synchronization setup when the sync pulse from a frame based camera is connected to
  an event camera. The tool compares the time stamp of the trigger event with that of the image frame, and
  gives the time by which the trigger event is delayed with respect to the frame.
  This time is usually negative, meaning the trigger event occurs before the frame based camera driver puts a header stamp on the image. A typical output will look like this:

  ```text
  [INFO] [1763569527.811228122] [trigger_delay]: frame rate:   1.000 Hz, trigger rate:   1.000 Hz, trigger delay: -3.531 ms
  [INFO] [1763569528.811002310] [trigger_delay]: frame rate:   1.000 Hz, trigger rate:   1.000 Hz, trigger delay: -3.868 ms
  ```

- ``ros2 run event_camera_tools event_rate [-b bag] [-r <rate_file>] [-t <trigger_file>] [-p period_ns] <ros_topic>``

  Counts the number of events in a fixed interval of length ``period`` (in nanoseconds), and writes it to ``rate_file``.
  The first column is the ROS time stamp, the second sensor time, then the number of OFF and ON events, and the number of UP
  and DOWN edge external trigger events. The external trigger events are also written to a separate file (``trigger_file``) with
  the exact time stamps and whether it's an UP (1) or DOWN (0) edge. Can also operate on a rosbag if provided with the ``-b`` option.

### Recorded data analysis tools

- ``ros2 run event_camera_tools bag_to_raw -t <topic> -b <bag_name> -o <outout_raw_file> -c <camera_type>``

  Converts bags with evt3 event_camera_msgs to raw file. The ``camera_type`` argument is necessary to produce a valid header for
  the raw file.
- ``ros2 run event_camera_tools raw_to_bag -t <topic> -b <bag_file> -i <input_raw_file> -w <sensor_width> -h <sensor_height> -p <packet_duration_ms>``

  Converts raw file into bag with evt3 event_camera_msgs. The ``packet_duration_ms`` gives the time slice (in  milliseconds) per ROS packet. The default is 10ms. Choose this parameter smaller to get lower processing latencies.

- ``ros2 run event_camera_tools movie_maker -f <fps> -b <bag_name> -t <topic>``

  Produces sequence of frame images.

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

  The ROS and sensor times of the first trigger event serve as inputs for [image reconstruction tools](https://github.com/ros-event-camera/event_image_reconstruction_fibar) such as ``bag_to_frames``.

- ``ros2 run event_camera_tools plot_events -b name_of_bag -o name_of_plot_file -t <topic> [-z]``

  Creates file with 5 columns (sensor time, ros time, x, y, polarity) for plotting. Also useful for clear text viewing of events in bag file.

- ``ros2 run event_camera_tools event_statics -b name_of_bag -t <topic> [-s <scale_file>]``

  Computes the per-pixel number of ON and OFF events in a bag and writes them to ``scale_file.txt`` (default).
  The file has alternatingly the number of OFF and ON events in row major order, with line breaks after each row.

## License

This software is issued under the Apache License Version 2.0.
