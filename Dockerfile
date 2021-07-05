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

FROM nanosaur/nanosaur_base:1.2

ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src

# Clone and build missing packages
RUN cd ${ROS_ROOT}/src && \
    git clone --branch dashing-devel https://github.com/ros/xacro.git && \
    git clone --branch ros2 https://github.com/ros/urdf_parser_py.git && \
    git clone --branch foxy https://github.com/ros/joint_state_publisher.git && \
    cd ${ROS_ROOT} && \
    colcon build --symlink-install --packages-select xacro urdfdom_py joint_state_publisher
# Copy wstool robot.rosinstall
COPY robot.rosinstall robot.rosinstall
# Initialize ROS2 workspace
RUN pip3 install wheel && \
    pip3 install -U wstool && \
    wstool init $ROS_WS/src && \
    wstool merge -t $ROS_WS/src robot.rosinstall && \
    wstool update -t $ROS_WS/src
# Copy nanosaur project
# COPY . $ROS_WS/src/nanosaur
# Install python dependencies
RUN apt-get update && \
    apt-get install libjpeg-dev zlib1g-dev python3-pip -y && \
    pip3 install -r $ROS_WS/src/nanosaur_robot/nanosaur_hardware/requirements.txt && \
    rm -rf /var/lib/apt/lists/*
# Change workdir
WORKDIR $ROS_WS
# build ros package source
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release
# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh
# run ros package launch file
CMD ["ros2", "launch", "nanosaur_bringup", "bringup.launch.py"]