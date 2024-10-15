#!/bin/bash

S_UID=${S_UID:-1001}
S_GID=${S_GID:-1001}

CONT_UID=$(id -u runner_user)
CONT_GID=$(id -g runner_user)

if [[ $CONT_UID -ne $S_UID ]] || [[ $CONT_GID -ne $S_GID ]]; then
    echo "Updating $USERNAME with UID:$S_UID and GID:$S_GID"
    # Update user and group ID to match S
    groupmod -g $S_GID runner_user
    usermod -u $S_UID -g $S_GID runner_user
    chown -R "$S_UID:$S_GID" /home/runner_user
fi

exec gosu runner_user "$@"
