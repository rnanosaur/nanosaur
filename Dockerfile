# Copyright (C) 2022, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# https://hub.docker.com/_/ros
FROM ros:foxy-ros-base

ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# Download and build nanosaur_ws
ENV ROS_WS /opt/ros_ws
# Copy wstool robot.rosinstall
COPY nanosaur/rosinstall/robot.rosinstall robot.rosinstall
# Initialize ROS2 workspace and install python dependencies
# fonts-open-sans required for nanosaur_base
RUN mkdir -p $ROS_WS/src && \
    vcs import $ROS_WS/src < robot.rosinstall && \
    apt-get update && \
    apt-get install libjpeg-dev zlib1g-dev python3-pip fonts-open-sans -y && \
    pip3 install -U jetson-stats && \
    pip3 install -r $ROS_WS/src/nanosaur_robot/nanosaur_base/requirements.txt && \
    rosdep install --from-paths $ROS_WS/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Change workdir
WORKDIR $ROS_WS

# ROS2 system manager docker build
ENV DOCKER_CONTAINER Yes
# Build Isaac ROS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# https://docs.docker.com/engine/reference/builder/#stopsignal
# https://hynek.me/articles/docker-signals/
STOPSIGNAL SIGINT
# run ros package launch file
CMD ["ros2", "launch", "nanosaur_bringup", "bringup.launch.py"]