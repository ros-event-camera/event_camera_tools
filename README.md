# event_array_tools

This repository holds tools for displaying and converting event_array_msgs under ROS and ROS2

## Supported platforms

Currently tested on Ubuntu 20.04 under under ROS Noetic and ROS2 Galactic.


## How to build
Create a workspace (``event_array_tools_ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
mkdir -p ~/event_array_tools_ws/src
cd ~/event_array_tools_ws
git clone https://github.com/berndpfrommer/event_array_tools.git src/event_array_tools
wstool init src src/event_array_tools/event_array_tools.rosinstall
# to update an existing space:
# wstool merge -t src src/event_array_tools/event_array_tools.rosinstall
# wstool update -t src
```

### configure and build on ROS1:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

### configure and build on ROS2:

```
cd ~/event_array_tools_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
```

## Tools

### Converting bags to and from raw

ROS1 examples:
```
rosrun metavision_ros_tools bag_to_raw -t /event_camera/events -b foo.bag -o foo.raw -c silkyev
rosrun metavision_ros_tools raw_to_bag -t /event_camera/events -i foo.raw -b foo.bag -f frame_id -w width -h height -e evt3
```

ROS2 examples:
```
ros2 run metavision_ros_tools bag_to_raw -t /event_camera/events -b foo.bag -o foo.raw -c silkyev
ros2 run metavision_ros_tools raw_to_bag -t /event_camera/events -i foo.raw -b foo.bag -f frame_id -w width -h height -e evt3
```

### Displaying rate and latency statistics

How to use (ROS1):
```
rosrun event_array_tools perf <ros_topic_of_events_here>
```

How to use (ROS2):
```
ros2 run event_array_tools perf <ros_topic_of_events_here>
```

This should give an output like this.
```
events:  33.3294 M/s msgs:   879.98/s drop:   3 delay:  0.50ms
events:  33.5153 M/s msgs:   882.50/s drop:   4 delay:  0.59ms
events:  33.5654 M/s msgs:   882.00/s drop:   2 delay:  0.73ms
events:  33.5919 M/s msgs:   886.50/s drop:   1 delay:  0.76ms
events:   9.2681 M/s msgs:   240.49/s drop:   1 delay:  0.86ms
```
 The important part is
"delay". It gives the average time difference between when the message
arrived at the subscriber (``perf``), and the stamp that is put on by the
driver (header.stamp in the message). The delay should be positive and
between 0 and 1 ms. Give the driver a bit of time after startup to
figure out how to set the timestamps right.
Also important: "drop" is the number of ROS packets that where
published by the driver but not received by ``perf``.

## License

This software is issued under the Apache License Version 2.0.
