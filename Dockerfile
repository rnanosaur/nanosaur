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
FROM dustynv/ros:foxy-ros-base-l4t-r32.6.1
# Configuration CUDA
ARG CUDA=10.2
ARG L4T=r32.6
ARG TENSORRT=8

ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# Clone and build missing packages
# to skip rosdep install --from-paths src --ignore-src -r -y
RUN mkdir -p ${ROS_ROOT}/src && \
    cd ${ROS_ROOT}/src && \
    git clone --branch ros2 https://github.com/ros/xacro.git && \
    git clone --branch ros2 https://github.com/ros/urdf_parser_py.git && \
    git clone --branch foxy https://github.com/ros/joint_state_publisher.git && \
    cd ${ROS_ROOT} && \
    . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    colcon build --symlink-install --merge-install --packages-select xacro urdfdom_py joint_state_publisher

# Download and build nanosaur_ws
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
# Copy wstool robot.rosinstall
COPY robot.rosinstall robot.rosinstall
# Initialize ROS2 workspace
RUN pip3 install wheel && \
    pip3 install -U wstool && \
    pip3 install jetson-stats && \
    wstool init $ROS_WS/src && \
    wstool merge -t $ROS_WS/src robot.rosinstall && \
    wstool update -t $ROS_WS/src

# Copy nanosaur project
# COPY . $ROS_WS/src/nanosaur
# Install python dependencies
RUN apt-get update && \
    apt-get install libjpeg-dev zlib1g-dev python3-pip -y && \
    pip3 install -r $ROS_WS/src/nanosaur_robot/nanosaur_base/requirements.txt && \
    rm -rf /var/lib/apt/lists/*

# Build on CUDA
# Copy and run jetson_utils installer
COPY jetson_cuda.sh /opt/jetson_cuda.sh
# Pass in order
# CUDA ex. 10.2
# L4T version ex. r32.5
# TENSORRT ex. 7
RUN . /opt/jetson_cuda.sh ${CUDA} ${L4T} ${TENSORRT}

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh
# Set default RMW implementation
# Fix camera run
# https://github.com/ros2/rmw_fastrtps/issues/460
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# run ros package launch file
CMD ["ros2", "launch", "nanosaur_bringup", "bringup.launch.py"]
# Change workdir
WORKDIR $ROS_WS