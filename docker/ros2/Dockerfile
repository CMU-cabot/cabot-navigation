# Copyright (c) 2020, 2024  Carnegie Mellon University and Miraikan
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

ARG BASE_IMAGE=cabot-base
ARG FROM_IMAGE=cmucal/${BASE_IMAGE}:humble-desktop-custom-mesa

# build underlay
FROM ${FROM_IMAGE} as cache

ENV UNDERLAY_WS=/opt/underlay_ws

RUN mkdir -p $UNDERLAY_WS/src
COPY ./grid_map $UNDERLAY_WS/src/grid_map
COPY ./tf2_web_republisher $UNDERLAY_WS/src/tf2_web_republisher
COPY ./rosbridge_suite $UNDERLAY_WS/src/rosbridge_suite

WORKDIR $UNDERLAY_WS

# copy package.xml files with parent directories for cache
RUN find . -name "src" -type d \
      -mindepth 1 -maxdepth 2 -printf '%P\n' \
      | xargs -I % mkdir -p /tmp/opt/% && \
    find . -name "package.xml" \
      | xargs cp --parents -t /tmp/opt && \
    find . -name "COLCON_IGNORE" \
      | xargs cp --parents -t /tmp/opt || true

RUN apt update && rosdep update && rosdep install -q -y \
	--from-paths src \
	--ignore-src

RUN . /opt/custom_ws/install/setup.sh && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo


FROM ${FROM_IMAGE} as build
ENV ROS_DISTRO=humble
ARG TZ=UTC
ENV TZ=$TZ
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | sudo tee /etc/timezone
ENV UNDERLAY_WS=/opt/underlay_ws

WORKDIR $UNDERLAY_WS

COPY --from=cache $UNDERLAY_WS/install $UNDERLAY_WS/install
# copy package.xml files from cache
COPY --from=cache /tmp/opt/src $UNDERLAY_WS/src

RUN apt update && rosdep update && rosdep install -q -y \
	--from-paths src \
	--ignore-src

ARG ROS_DISTRO

# TODO replace with rosdep install
RUN apt update && \
    apt install -y --no-install-recommends \
    alsa-utils \
    bc \
    ffmpeg \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev \
    libyaml-cpp-dev \
    libomp-dev \
    libpcl-dev \
    libasound2 \
    python3-pykdl \
    python3-sklearn \
    python3-venv \
    qtbase5-dev \
    ros-$ROS_DISTRO-diagnostic-aggregator \
    ros-$ROS_DISTRO-gazebo-msgs \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-rosidl-runtime-cpp \
    ros-$ROS_DISTRO-rtcm-msgs \
# pcl-ros is not ported yet
# #    ros-$ROS_DISTRO-pcl-msgs \
# #    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-geographic-msgs \
    libgeographic-dev \
# workaround end
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-ros-testing \
    ros-$ROS_DISTRO-rqt-robot-monitor \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    xterm \
    gosu \
    && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    # install ipython==8.0.1 to prevent errors while using odrivetool
    ipython==8.0.1 \
    odrive==0.6.5 \
    numpy==1.24.4 \
    pydub \
    pyproj \
    pyserial \
    python-can \
    requests \
    transforms3d==0.4.1
# for pedestrian test
RUN pip3 install --no-cache-dir \
    'pysocialforce[test,plot]' \
    # for test aggregation
    pandas

# for debug
# RUN apt update && \
#     apt install -y --no-install-recommends \
#     gdb \
#     vim \
#     && \
#     rm -rf /var/lib/apt/lists/*

# set up venv for keeping compatibility
RUN mkdir -p /opt/venv
# cabot2 venv (odrive==0.5.2.post0)
RUN python3 -m venv --system-site-packages /opt/venv/cabot2 && \
    . /opt/venv/cabot2/bin/activate && \
    pip3 install --no-cache-dir odrive==0.5.2.post0 && \
    deactivate
# cabot3 venv (odrive==0.6.5)
RUN python3 -m venv --system-site-packages /opt/venv/cabot3

ENV USERNAME developer
# Replace 1000 with your user/group id
ARG UID=1000
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
	usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d/ && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid $UID $USERNAME && \
        groupmod --gid $UID $USERNAME
# add user to video group to fix NvRmMemInitNvmap error on Jetson
# https://forums.developer.nvidia.com/t/jetpack-5-0-dp-nvrmmeminitnvmap-failed-with-permission-denied/219804
RUN usermod -aG video $USERNAME

# install entrypoint
COPY launch.sh /
#COPY ros_entrypoint.sh /ros_entrypoint.sh

# change location of ros base
RUN sed -i 's:custom_ws:underlay_ws:' /ros_entrypoint.sh

USER $USERNAME

ENV HOME /home/$USERNAME

WORKDIR $HOME/ros2_ws

RUN mkdir -p $HOME/ros2_ws/src && \
    mkdir -p $HOME/ros2_ws/script

COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot src/cabot
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot_bt src/cabot_bt
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-description/cabot_description src/cabot_description
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot_diagnostics src/cabot_diagnostics
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot_gazebo src/cabot_gazebo
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot_navigation2 src/cabot_navigation2
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot_ui src/cabot_ui
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./mf_localization_msgs src/mf_localization_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./mf_localization_rviz src/mf_localization_rviz
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./pedestrian_plugin src/pedestrian_plugin
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./pedestrian_plugin_msgs src/pedestrian_plugin_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-common/cabot_common src/cabot_common
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-common/cabot_msgs src/cabot_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./queue_msgs src/queue_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./queue_people_py src/queue_people_py
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./queue_utils_py src/queue_utils_py
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./script script

RUN /ros_entrypoint.sh /launch.sh build

FROM ${FROM_IMAGE} as final
ENV ROS_DISTRO=humble
ARG TZ=UTC
ENV TZ=$TZ
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | sudo tee /etc/timezone

ENV UNDERLAY_WS=/opt/underlay_ws

WORKDIR $UNDERLAY_WS

COPY --from=cache $UNDERLAY_WS/install $UNDERLAY_WS/install
# copy package.xml files from cache
COPY --from=cache /tmp/opt/src $UNDERLAY_WS/src

RUN apt update && rosdep update && rosdep install -q -y \
	--from-paths src \
	--ignore-src

ARG ROS_DISTRO

# TODO replace with rosdep install
RUN apt update && \
    apt install -y --no-install-recommends \
    alsa-utils \
    bc \
    ffmpeg \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev \
    libyaml-cpp-dev \
    libomp-dev \
    libpcl-dev \
    libasound2 \
    python3-pykdl \
    python3-sklearn \
    python3-venv \
    qtbase5-dev \
    ros-$ROS_DISTRO-diagnostic-aggregator \
    ros-$ROS_DISTRO-gazebo-msgs \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-rosidl-runtime-cpp \
# pcl-ros is not ported yet
# #    ros-$ROS_DISTRO-pcl-msgs \
# #    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-geographic-msgs \
    libgeographic-dev \
# workaround end
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-ros-testing \
    ros-$ROS_DISTRO-rqt-robot-monitor \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    xterm \
    gosu \
    && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    # install ipython==8.0.1 to prevent errors while using odrivetool
    ipython==8.0.1 \
    odrive==0.6.5 \
    numpy==1.24.4 \
    pydub \
    pyproj \
    pyserial \
    requests \
    transforms3d==0.4.1
# for pedestrian test
RUN pip3 install --no-cache-dir \
    'pysocialforce[test,plot]'

# for debug
# RUN apt update && \
#     apt install -y --no-install-recommends \
#     gdb \
#     vim \
#     && \
#     rm -rf /var/lib/apt/lists/*

# set up venv for keeping compatibility
RUN mkdir -p /opt/venv
# cabot2 venv (odrive==0.5.2.post0)
RUN python3 -m venv --system-site-packages /opt/venv/cabot2 && \
    . /opt/venv/cabot2/bin/activate && \
    pip3 install --no-cache-dir odrive==0.5.2.post0 && \
    deactivate
# cabot3 venv (odrive==0.6.5)
RUN python3 -m venv --system-site-packages /opt/venv/cabot3

ENV USERNAME developer
# Replace 1000 with your user/group id
ARG UID=1000
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
	usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d/ && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid $UID $USERNAME && \
        groupmod --gid $UID $USERNAME
# add user to video group to fix NvRmMemInitNvmap error on Jetson
# https://forums.developer.nvidia.com/t/jetpack-5-0-dp-nvrmmeminitnvmap-failed-with-permission-denied/219804
RUN usermod -aG video $USERNAME

# install entrypoint
COPY launch.sh /

# change location of ros base
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sed -i 's:custom_ws:underlay_ws:' /ros_entrypoint.sh

ENV HOME /home/$USERNAME

WORKDIR $HOME/ros2_ws
RUN chown $USERNAME:$USERNAME $HOME/ros2_ws

COPY --chown=$USERNAME:$USERNAME --from=build $HOME/ros2_ws/install $HOME/ros2_ws/install
COPY --chown=$USERNAME:$USERNAME --from=build $HOME/ros2_ws/script $HOME/ros2_ws/script

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "/launch.sh" ]

