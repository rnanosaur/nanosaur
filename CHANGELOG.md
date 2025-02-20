# Changelog

## [2.0.0-pre2]

### Features

- Added a special Dockerfile for debugging called `nanosaur/nanosaur:diagnostic` built multi-platform
- Improve [`nanosaur_simulation`](https://github.com/rnanosaur/nanosaur_simulation) to support different word load
- New docker in [`nanosaur_simulation`](https://github.com/rnanosaur/nanosaur_simulation) to convert from sdf to udf (waiting merge: <https://github.com/gazebosim/gz-usd/pull/10> )

### Fixes

- `docker-compose.yml` improved

## [2.0.0-pre1] - 2025-02-15

Improve CI and align [`nanosaur_robot`](https://github.com/rnanosaur/nanosaur_robot) on the new `nanosaur2` branch.

## [2.0.0-pre0] - 2025-02-12

New nanosaur and fully reshape of the nanosaur code. This new release works only in Simulation.

### Features

- New nanosaur STL
- New controllers
- Added new package `nanosaur_visualization`
- In [`nanosaur_perception`](https://github.com/rnanosaur/nanosaur_perception) rewrite all launcher and move to Isaac ROS 3.2
- Improve and updated [`nanosaur_simulation`](https://github.com/rnanosaur/nanosaur_simulation) now support
  - Isaac Sim 4.5
  - Automatic load the robot from URDF file

## [1.7.0] - 2022-09-06

### Fixes

- Bump docker/build-push-action from 3.0.0 to 3.1.0 by @dependabot in #78
- Bump docker/build-push-action from 3.1.0 to 3.1.1 by @dependabot in #79

## [1.6.2] - 2022-07-05

### Fixes

- Bump dependabot/fetch-metadata from 1.3.1 to 1.3.2 by @dependabot in #76
- Bump dependabot/fetch-metadata from 1.3.2 to 1.3.3 by @dependabot in #77

## [1.6.1] - 2022-06-19

Renamed version from nanosaur 2.1

## [1.6.0] - 2022-06-08

Renamed version from nanosaur 2.0

### Features

- Fully Isaac ROS integration
- New repository for perception [nanosaur_perception](https://github.com/rnanosaur/nanosaur_perception)
- Release cover for:
  - Realsense d435
  - ZED mini camera
- New documentation

### Fixes

- Bump actions/checkout from 2.3.5 to 2.4.0 by @dependabot in #33
- Bump crazy-max/ghaction-docker-meta from 3.6.0 to 3.6.2 by @dependabot in #42
- Bump docker/login-action from 1.10.0 to 1.12.0 by @dependabot in #45
- Fix /etc/docker/daemon.json download error by @Tiryoh in #47
- Bump docker/build-push-action from 2.7.0 to 2.9.0 by @dependabot in #50
- Bump actions/checkout from 2.4.0 to 3 by @dependabot in #53
- Bump docker/login-action from 1.12.0 to 1.14.1 by @dependabot in #54
- Bump docker/build-push-action from 2.9.0 to 2.10.0 by @dependabot in #55
- Bump peter-evans/dockerhub-description from 2 to 3 by @dependabot in #56
- Bump crazy-max/ghaction-docker-meta from 3.6.2 to 3.7.0 by @dependabot in #58
- Bump actions/upload-artifact from 2 to 3 by @dependabot in #63
- Bump docker/build-push-action from 2.7.0 to 2.10.0 by @dependabot in #64
- Bump docker/login-action from 1.10.0 to 1.14.1 by @dependabot in #65
- Bump actions/download-artifact from 2 to 3 by @dependabot in #66
- Bump crazy-max/ghaction-docker-meta from 3.6.0 to 3.7.0 by @dependabot in #67
- Bump dependabot/fetch-metadata from 1.1.1 to 1.3.1 by @dependabot in #69
- Bump docker/metadata-action from 3 to 4 by @dependabot in #70
- Bump docker/build-push-action from 2.10.0 to 3.0.0 by @dependabot in #71
- Bump docker/setup-buildx-action from 1 to 2 by @dependabot in #72
- Bump docker/setup-qemu-action from 1 to 2 by @dependabot in #73
- Bump docker/login-action from 1.14.1 to 2.0.0 by @dependabot in #74

## [1.6.0-dev3] - 2022-04-20

### Fixes

- Bump actions/upload-artifact from 2 to 3 by @dependabot in #63
- Bump docker/build-push-action from 2.7.0 to 2.10.0 by @dependabot in #64
- Bump docker/login-action from 1.10.0 to 1.14.1 by @dependabot in #65
- Bump actions/download-artifact from 2 to 3 by @dependabot in #66
- Bump crazy-max/ghaction-docker-meta from 3.6.0 to 3.7.0 by @dependabot in #67

## [1.6.0-dev2] - 2022-04-08

Minor fixes

## [1.6.0-dev1] - 2022-04-08

### Fixes

- Bump actions/checkout from 2.3.5 to 2.4.0 by @dependabot in #33
- Bump crazy-max/ghaction-docker-meta from 3.6.0 to 3.6.2 by @dependabot in #42
- Bump docker/login-action from 1.10.0 to 1.12.0 by @dependabot in #45
- Fix /etc/docker/daemon.json download error by @Tiryoh in #47
- Bump docker/build-push-action from 2.7.0 to 2.9.0 by @dependabot in #50
- Bump actions/checkout from 2.4.0 to 3 by @dependabot in #53
- Bump docker/login-action from 1.12.0 to 1.14.1 by @dependabot in #54
- Bump docker/build-push-action from 2.9.0 to 2.10.0 by @dependabot in #55
- Bump peter-evans/dockerhub-description from 2 to 3 by @dependabot in #56
- Bump crazy-max/ghaction-docker-meta from 3.6.2 to 3.7.0 by @dependabot in #58

## [1.5.1] - 2021-10-30

### Features

- improved **nanosaur** script
  - added option **nanosaur domain** to change ROS_DOMAIN_ID
  - added option **nanosaur wakeup** to quickly wakeup nanosaur (eq. nanosaur up -d)
- Bump actions/checkout from 2.3.4 to 2.3.5 by @dependabot in #31
- Bump crazy-max/ghaction-docker-meta from 3.5.0 to 3.6.0 by @dependabot in #32

## [1.5.0] - 2021-10-15

A new stable version of nanosaur docker.

### Features

- New nanosaur script for jetson and desktop
- New docker-compose with three containers (core,WebGUIi andauto-updaterr)
- Reshape packages

## [1.4.2] - 2021-09-02

### Features

- New eyes controller. Finally the nanosaur display are working without show a dummy message
- Improved the camera controller and added the camera_info message
- Improved the installer, fix man bugs
- Improved nanosaur script. Now you can use the new command `nanosaur` from your NVIDIA Jetson or from your desktop machine
- Added a dockerfile for galactic

## [1.4.1] - 2021-08-14

From this release, Nanosaur works with **Jetpack 4.6**

remember to update your robot form [NVIDIA Jetpack](https://developer.nvidia.com/embedded/jetpack)

## [1.4.0] - 2021-08-13

Rearrange all nanosaur packages:

- nanosaur_hardware -> nanosaur_base
- Now you can edit all configuration of nanosaur from a yml file (inside or outside docker)
- nanosaur_camera fixed. Run and works on `rviz2` using `cyclonedds`
- New dockerfile starting from `dustynv/ros:foxy-ros-base-l4t-r32.5.0`
- Dockerfile size only 1.8Gb

## [1.3.1] - 2021-07-04

Fix github workflow

## [1.3.0] - 2021-07-04

General improvement on the nanosaur code

## [1.2.0] - 2021-04-07

### Fixes

- Default ball bearings F686ZZ

## [1.1.1] - 2021-04-07

### Fixes

- Fix `base_front.stl` error

## [1.1.0] - 2021-02-18

This version enlarge the dock area for different type of power banks, now the maximum size is: **66 mm x 94 mm x 24 mm or 2.5' x 3.7' x 0.9'**

STL changed:

- `base_rear.stl`
- `cover.stl`
- `pb_holder.stl`

## [1.0.0] - 2021-02-17

First nanosaur release
