# iLidar API C++ Release Notes

## Introduction

1. This document describes the software updates for the `iLidar API C++`.
2. For further details on the softwares, please contact **json@hybo.co** or **jungingyo@hybo.co**.

## Change Log

### [V1.12.5] - 2025-04-29

- Changed
  - Updated `iTFS-80.dat` to remove invalid data in `pcl` example

### [V1.12.4b] - 2025-04-18

- Added
  - Added build feature of `dll` and `streamer`
  - Edge rejection for `openCV example`
  - New commands for `helloworld example`
  - Packet modifications due to sensor firmware version up

- Changed
  - Changed some parameters for better visualization

### [V1.12.1] - 2024-11-06

- Added
  - add new packet for F/W V1.5.0+ (`info_v2`)
  - add new packet for F/W V1.5.5+ (`sync_ack`)
- Changed
  - update library version to V1.12.1
  - change max_device values to `8` for multi-LiDAR solutions
  - change `info_packet_hanlder` in example codes

### [V1.11.10] - 2024-01-15

#### Added

- API files (`ilidar.cpp`, `ilidar.hpp`, and `packet.hpp`) for packet and iTFS sensor.
- 3 basic examples
  - `helloworld`: check sensor status and change its configuration
  - `opencv`: depth and intensity image viewer based on `opencv`
  - `pcl`: 3D point cloud viewer based on `point cloud library`