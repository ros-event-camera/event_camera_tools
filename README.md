# event_camera_tools

This repository holds ROS/ROS2 tools for displaying and converting
[event_camera_msgs](https://github.com/ros-event-camera/event_camera_msgs)
under ROS and ROS2. These messages are produced by the
[metavision_driver](https://github.com/ros-event-camera/metavision_ros_driver).

## Supported platforms

Currently tested on Ubuntu 20.04/22.04 under ROS Noetic and ROS2 Galactic/Humble.

## How to build
Create a workspace (``event_camera_tools_ws``), clone this repo, and use ``vcs``
to pull in the remaining dependencies:

```
pkg=event_camera_tools
mkdir -p ~/${pkg}_ws/src
cd ~/${pkg}_ws
git clone https://github.com/ros-event-camera/${pkg}.git src/${pkg}
cd src
vcs import < ${pkg}/${pkg}.repos
cd ..
```

### configure and build on ROS1:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

### configure and build on ROS2:

```
cd ~/${pkg}_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
```

## Republish conversion nodelet

The ``republish`` node converts event_camera_msgs to legacy formats
like dvs, prophesee, and decoded ("mono") event_camera messages. Note
that this nodelet will be consuming a significant amount of CPU
resources and should not be run unnecessarily while recording data.
The following command will start a conversion nodelet to republish
events from ``/event_camera/events`` to
``/event_camera/republished_events`` and
``/event_camera/republished_triggers``
(similar syntax under ROS2, see launch file for remapping):
```
roslaunch event_camera_tools republish_nodelet.launch camera:=event_camera message_type:=event_packet
```

## ROS1/ROS2 Tools

All tools are provided in ROS and ROS2 (syntax below is for ROS):

- ``rosrun event_camera_tools echo [-b <bag>] <topic>``: displays messages in
  ``event_camera_msgs`` format, optionally from a bag file. Example output:
  ```
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
- ``rosrun event_camera_tools perf <topic>``:
  Sample output:
  ```
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
- ``rosrun event_camera_tools sync_test <cam_0_event_topic> <cam_1_event_topic>``
  The output gives the average sensor time difference and how many
  samples where counted:
  ```
  avg sensor diff:  0.00846s, count:   360
  avg sensor diff:  0.00377s, count:   466
  ...
  ```
- ``rosrun event_camera_tools bag_to_raw -t <topic> -b <bag_name> -o <outout_raw_file> -c <camera_type>``

  Converts bags with evt3 event_camera_msgs to raw file. The
  ``camera_type`` argument is necessary to produce a valid header for
  the raw file.
- ``rosrun event_camera_tools raw_to_bag -t <topic> -b <bag_file> -i <input_raw_file> -w <sensor_width> -h <sensor_height> -B <buffer_size>``

  Converts raw file into bag with evt3 event_camera_msgs. The buffer
  size determines the size and number of ROS messages in the bag.
- ``rosrun event_camera_tools movie_maker -f <fps> -b <bag_name> -t <topic>``

  produces sequence of frame images.
## ROS1 only tools:

- ``rosrun event_camera_tools legacy_to_bag -t <topic> -b <input_bag_file> -o <output_bag_file>``

  Converts bags with DVS or Prophesee messages to evt3 event_camera_msgs.


## ROS2 only tools

- ``ros2 run event_camera_tools find_trigger_events -i input_bag -t topic_with_trigger_events``

  This tool is useful when aligning reconstructed image frames for datasets that use a pulse-per-second synchronization scheme. The ``find_trigger_events`` tool finds the ROS time and sensor time of the first trigger pulse in a rosbag. Output example:
  ```
  first trigger ROS    time: 1702577874035860000
  first trigger sensor time: 24809136000
  num triggers: 1712
  avg time between triggers:
  ROS time:    0.0499567s
  sensor time: 0.0499567s
  processed 202 number of messages
  ```
  The ROS and sensor times of the first trigger event serve as inputs for [image reconstruction tools](https://github.com/berndpfrommer/simple_image_recon) such as ``bag_to_frames``.


- ``ros2 run event_camera_tools plot_events -b name_of_bag -o name_of_plot_file -t topic``

  Creates file with 4 columns (time, x, y, polarity) for plotting. Also useful for clear text viewing of events in bag file.

## License

This software is issued under the Apache License Version 2.0.
