#!/bin/bash
set -e

# Default values
HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}
HOST_TZ=${HOST_TZ:-UTC}

if [[ $TZ != $HOST_TZ ]]; then
    # Setting up timezone
    sudo ln -snf /usr/share/zoneinfo/$HOST_TZ /etc/localtime
    echo $HOST_TZ | sudo tee /etc/timezone
    export TZ=$HOST_TZ
fi

CONT_UID=$(id -u developer)
CONT_GID=$(id -g developer)
if [[ $CONT_UID -ne $HOST_UID ]] || [[ $CONT_GID -ne $HOST_GID ]]; then
    # Update user and group ID to match host
    sudo usermod -u $HOST_UID developer
    sudo groupmod -g $HOST_GID developer
fi

HF_HOME_PATH=${HF_HOME:-/opt/huggingface}
if [[ -d "$HF_HOME_PATH" ]]; then
    # The model cache is downloaded at build time. When developer UID/GID is remapped
    # at runtime, ownership may no longer match and model load can fail.
    sudo chown -R "$HOST_UID:$HOST_GID" "$HF_HOME_PATH"
fi

exec gosu developer "$@"
