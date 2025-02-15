#!/bin/bash
set -e

# Default values
HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}
HOST_TZ=${HOST_TZ:-Etc/UTC}

if [[ $TZ != $HOST_TZ ]]; then
    # Setting up timezone
    ln -snf /usr/share/zoneinfo/$HOST_TZ /etc/localtime
    echo $HOST_TZ | sudo tee /etc/timezone
    export TZ=$HOST_TZ
fi

CONT_UID=$(id -u runner_user)
CONT_GID=$(id -g runner_user)
if [[ $CONT_UID -ne $HOST_UID ]] || [[ $CONT_GID -ne $HOST_GID ]]; then
    # Update user and group ID to match host
    usermod -u $HOST_UID runner_user
    groupmod -g $HOST_GID runner_user
fi

exec gosu runner_user "$@"
