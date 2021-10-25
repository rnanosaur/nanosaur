# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
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

# Jetpack 4.6
# https://github.com/dusty-nv/jetson-containers
FROM dustynv/ros:galactic-ros-base-l4t-r32.6.1
# Configuration CUDA
ARG CUDA=10.2
ARG L4T=r32.6
ARG TENSORRT=8

ENV ROS_DISTRO=galactic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# Copy wstool docker.rosinstall
# to skip rosdep install --from-paths src --ignore-src -r -y
COPY nanosaur/rosinstall/docker.rosinstall docker.rosinstall
# Initialize ROS2 workspace
RUN apt-get update && \
    apt-get install python3-vcstool python3-pip -y && \
    pip3 install wheel && \
    pip3 install -U jetson-stats&& \
    mkdir -p ${ROS_ROOT}/src && \
    vcs import ${ROS_ROOT}/src < docker.rosinstall && \
    cd ${ROS_ROOT} && \
    . ${ROS_ROOT}/install/setup.sh && \
    colcon build --symlink-install --merge-install \
    --packages-select xacro urdfdom_py joint_state_publisher teleop_twist_joy joy sdl2_vendor diagnostic_updater twist_mux && \
    rm -rf /var/lib/apt/lists/*

# Download and build nanosaur_ws
ENV ROS_WS /opt/ros_ws
# Copy wstool robot.rosinstall
COPY nanosaur/rosinstall/robot.rosinstall robot.rosinstall
# Initialize ROS2 workspace and install python dependencies
RUN mkdir -p $ROS_WS/src && \
    vcs import $ROS_WS/src < robot.rosinstall && \
    apt-get update && \
    apt-get install libjpeg-dev zlib1g-dev -y && \
    pip3 install -r $ROS_WS/src/nanosaur_robot/nanosaur_base/requirements.txt && \
    rm -rf /var/lib/apt/lists/*

# Build on CUDA
# Copy and run jetson_utils installer
COPY nanosaur/scripts/jetson_cuda.sh /opt/jetson_cuda.sh
# Pass in order
# CUDA ex. 10.2
# L4T version ex. r32.5
# TENSORRT ex. 7
# ROS_DISTRO ex. foxy
# ROS_WS ex. /opt/ros_ws
RUN . /opt/jetson_cuda.sh ${CUDA} ${L4T} ${TENSORRT} ${ROS_DISTRO} ${ROS_WS}

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# https://docs.docker.com/engine/reference/builder/#stopsignal
# https://hynek.me/articles/docker-signals/
STOPSIGNAL SIGINT
# Set default RMW implementation
# Fix camera run
# https://github.com/ros2/rmw_fastrtps/issues/460
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# run ros package launch file
CMD ["ros2", "launch", "nanosaur_bringup", "bringup.launch.py"]
# Change workdir
WORKDIR $ROS_WS