^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package event_camera_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fix more c++ standard 20 warnings
* fix c++ standard 20 warnings
* Contributors: Bernd Pfrommer

3.1.3 (2026-04-26)
------------------
* fix c++ standard 20 warnings
* added seqno gap detection to event_statistics
* store in mcap mode (also on humble)
* provide timestamps file to correlate frame numbers with time stamps
* add dependency on ament_cmake_test for humble builds
* Contributors: Bernd Pfrommer

3.1.2 (2026-04-07)
------------------
* rely on event_camera_codecs and _msgs packages
* Remove unused include of rosbag2_cpp/typesupport_helpers.hpp
  This header file was removed in Jazzy (rosbag2 release 0.26.10). Since
  nothing seems to use this header file, let's remove it.
  This fixes the following compile error:
  In file included from src/raw_to_bag.cpp:20:
  include/event_camera_tools/ros_compat.h:29:10: fatal error:
  rosbag2_cpp/typesupport_helpers.hpp: No such file or directory
  29 | #include <rosbag2_cpp/typesupport_helpers.hpp>
  |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* added sanity checks for event_statistics
* use header for raw_to_bag
* improve trigger delay, compute stddev of delay
* added jitter measurement
* Contributors: Bernd Pfrommer, Michal Sojka

3.1.1 (2025-12-08)
------------------
* Merge branch 'master' into release
* added opencv dependency to package file
* Contributors: Bernd Pfrommer

3.1.0 (2025-12-08)
------------------
* first release
* Contributors: Bernd Pfrommer, Dhruv Patel, YLFeng
