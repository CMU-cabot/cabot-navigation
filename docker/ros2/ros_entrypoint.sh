#!/bin/bash
set -e

# Default values
HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}
HOST_TZ=${HOST_TZ:-UTC}

# Setting up timezone
sudo ln -snf /usr/share/zoneinfo/$HOST_TZ /etc/localtime
echo $HOST_TZ | sudo tee /etc/timezone

# Update user and group ID to match host
sudo usermod -u $HOST_UID developer
sudo groupmod -g $HOST_GID developer

# Source ROS setup script
source "/opt/custom_ws/install/setup.bash"
exec "$@"
