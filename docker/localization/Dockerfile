ARG BASE_IMAGE=cabot-base
ARG FROM_IMAGE=cmucal/${BASE_IMAGE}:humble-desktop-custom-mesa

########## cache ##########
FROM ${FROM_IMAGE} as cache

ENV CARTO_WS=/opt/cartographer_ws

RUN mkdir -p $CARTO_WS/src
COPY ./cartographer /opt/cartographer_ws/src/cartographer
COPY ./cartographer_ros /opt/cartographer_ws/src/cartographer_ros
COPY ./bluespace_ai_xsens_ros_mti_driver /opt/cartographer_ws/src/bluespace_ai_xsens_ros_mti_driver
COPY ./ublox /opt/cartographer_ws/src/ublox
COPY ./ntrip_client /opt/cartographer_ws/src/ntrip_client

WORKDIR /opt/cartographer_ws/

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

## TODO: this is a quick fix
RUN sed -i 's:focal:jammy:' src/cartographer/scripts/install_debs_cmake.sh
RUN sed -i 's:d902eb869bcfacc1bad14933ed9af4bed006d481:8c0b94e793a66495e0b1f34a5eb26bd7dc672db0:' src/cartographer/scripts/install_abseil.sh
## TODO end

RUN src/cartographer/scripts/install_debs_cmake.sh && rm -rf /var/lib/apt/lists/*
RUN src/cartographer/scripts/install_abseil.sh && rm -rf /var/lib/apt/lists/*

RUN . /opt/custom_ws/install/setup.sh && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# protoc
RUN mkdir -p /opt/cartographer_proto
RUN protoc -I./src/cartographer --python_out=/opt/cartographer_proto $(find src/cartographer/ -name '*.proto')

########## build ##########
FROM $FROM_IMAGE as build

ARG TZ=UTC
ENV TZ=$TZ
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | sudo tee /etc/timezone

ENV CARTO_WS=/opt/cartographer_ws

RUN mkdir -p $CARTO_WS/install
WORKDIR /opt/cartographer_ws/

COPY --from=cache $CARTO_WS/install $CARTO_WS/install
# copy package.xml files from cache
COPY --from=cache /tmp/opt/src $CARTO_WS/src

RUN apt update && rosdep update && rosdep install -q -y \
	--from-paths src \
	--ignore-src

ARG ROS_DISTRO
RUN sed -i 's:custom_ws:cartographer_ws:' /ros_entrypoint.sh


# TODO replace with rosdep install
RUN apt update && \
    apt install -y --no-install-recommends \
    imagemagick \
    libpcap-dev \
    nodejs \
    npm \
    python3-pip \
    python3-matplotlib \
    python3-sklearn \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-velodyne-description \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-robot-localization \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
    requests \
    certifi \
    orjson==3.10.6 \
    pyproj \
    rosbags

## install cvi client packages  # TODO still needed?
RUN apt update && \
    apt install -y --no-install-recommends \
    python3-tk \
    python-is-python3 \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

## install packages for ublox node and RTKLIB
RUN apt update && \
    apt-get install -y \
    libasio-dev \
    ros-${ROS_DISTRO}-rtcm-msgs \
    ros-${ROS_DISTRO}-nmea-msgs \
    gfortran \
    && \
    rm -rf /var/lib/apt/lists/*

## install RTKLIB
WORKDIR /opt
COPY ./RTKLIB/ ./RTKLIB/
RUN (cd RTKLIB/lib/iers/gcc/; make) && \
    (cd RTKLIB/app/consapp; make; make install) && \
    rm -rf RTKLIB

# TODO merge
RUN pip install --no-cache-dir \
	transforms3d==0.4.1 \
	pyserial
RUN apt update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    xterm \
    && \
    rm -rf /var/lib/apt/lists/*
# end TODO

COPY ./launch.sh /
COPY ./launch_rtklib.sh /

# setup for display
ENV USERNAME developer

### replace 1000 with your user/group id
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

USER $USERNAME

ENV HOME /home/$USERNAME
WORKDIR $HOME/loc_ws

RUN mkdir -p $HOME/loc_ws/src && \
    mkdir -p $HOME/loc_ws/script

COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-description/cabot_description src/cabot_description
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-common/cabot_common src/cabot_common
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot-common/cabot_msgs src/cabot_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot_mf_localization src/cabot_mf_localization
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./script script
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./mf_localization src/mf_localization
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./mf_localization_gazebo src/mf_localization_gazebo
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./mf_localization_mapping src/mf_localization_mapping
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./mf_localization_msgs src/mf_localization_msgs
COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./mf_localization_rviz src/mf_localization_rviz

RUN /ros_entrypoint.sh /launch.sh build

########## final ##########
FROM $FROM_IMAGE as final

ARG TZ=UTC
ENV TZ=$TZ
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | sudo tee /etc/timezone

ENV CARTO_WS=/opt/cartographer_ws

RUN mkdir -p $CARTO_WS/install
WORKDIR /opt/cartographer_ws/

COPY --from=cache $CARTO_WS/install $CARTO_WS/install
# copy package.xml files from cache
COPY --from=cache /tmp/opt/src $CARTO_WS/src

RUN apt update && rosdep update && rosdep install -q -y \
	--from-paths src \
	--ignore-src

ARG ROS_DISTRO
RUN sed -i 's:custom_ws:cartographer_ws:' /ros_entrypoint.sh

# protoc
RUN mkdir -p /opt/cartographer_proto
COPY --from=cache /opt/cartographer_proto /opt/cartographer_proto
ENV PYTHONPATH /opt/cartographer_proto:$PYTHONPATH

# TODO replace with rosdep install
RUN apt update && \
    apt install -y --no-install-recommends \
    imagemagick \
    libpcap-dev \
    nodejs \
    npm \
    python3-pip \
    python3-matplotlib \
    python3-pandas \
    python3-sklearn \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-velodyne-description \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-robot-localization \
    gosu \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
    requests \
    certifi \
    orjson \
    pyproj \
    rosbags

## install cvi client packages  # TODO still needed?
RUN apt update && \
    apt install -y --no-install-recommends \
    python3-tk \
    python-is-python3 \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

## install packages for ublox node and RTKLIB
RUN apt update && \
    apt-get install -y \
    libasio-dev \
    ros-${ROS_DISTRO}-rtcm-msgs \
    ros-${ROS_DISTRO}-nmea-msgs \
    gfortran \
    && \
    rm -rf /var/lib/apt/lists/*

## install RTKLIB
WORKDIR /opt
COPY ./RTKLIB/ ./RTKLIB/
RUN (cd RTKLIB/lib/iers/gcc/; make) && \
    (cd RTKLIB/app/consapp; make; make install) && \
    rm -rf RTKLIB

# TODO merge
RUN pip install --no-cache-dir \
    transforms3d==0.4.1 \
    pyserial \
    # pbstream edit
    imageio \
    tifffile \
    protobuf==3.20.3
RUN apt update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    xterm \
    && \
    rm -rf /var/lib/apt/lists/*
# end TODO

COPY ./launch.sh /
COPY ./launch_rtklib.sh /

# setup for display
ENV USERNAME developer

### replace 1000 with your user/group id
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

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sed -i 's:custom_ws:cartographer_ws:' /ros_entrypoint.sh

ENV HOME /home/$USERNAME

WORKDIR $HOME/loc_ws
RUN chown $USERNAME:$USERNAME $HOME/loc_ws

COPY --chown=$USERNAME:$USERNAME --from=build $HOME/loc_ws/install $HOME/loc_ws/install
COPY --chown=$USERNAME:$USERNAME --from=build $HOME/loc_ws/script $HOME/loc_ws/script
COPY --chown=$USERNAME:$USERNAME --from=build $HOME/loc_ws/src/cabot_mf_localization/script $HOME/loc_ws/src/cabot_mf_localization/script
#COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./script script
#COPY --chown=$USERNAME:$USERNAME --from=cabot_src ./cabot_mf_localization/script src/cabot_mf_localization/script

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "/launch.sh" ]
