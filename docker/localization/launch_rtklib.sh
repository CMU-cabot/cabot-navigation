#!/bin/bash

# Copyright (c) 2021  IBM Corporation
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

# functions
function red {
    echo -en "\033[31m"  ## red
    echo $1
    echo -en "\033[0m"  ## reset color
}

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

# environment variables
# ntrip client parameters
: ${GNSS_NODE_START_AT_LAUNCH:=1}
: ${NTRIP_CLIENT_START_AT_LAUNCH:=0}
: ${NTRIP_CLIENT:=ntrip_client}
: ${NTRIP_HOST:=}
: ${NTRIP_PORT:=}
: ${NTRIP_MOUNTPOINT:=}
: ${NTRIP_AUTHENTIFICATE:=}
: ${NTRIP_USERNAME:=}
: ${NTRIP_STR2STR_RELAY_BACK:=0}

# source local workspace
source install/setup.bash

# RTKLIB
if [ "${NTRIP_CLIENT}" = "rtklib" ]; then
    # if RTK_STR_IN exists in environment variables, use it.
    # if not, load RTK_STR_IN from CABOT_SITE package
    if [ "${RTK_STR_IN}" != "" ]; then
        echo "RTK_STR_IN=${RTK_STR_IN}"
    else
        if [ "${CABOT_SITE}" != "" ]; then
        sitedir=`ros2 pkg prefix $CABOT_SITE`/share/$CABOT_SITE
            source $sitedir/config/rtk_config.sh
        fi
    fi

    # check if RTK_STR_IN is defined.
    if [ "${RTK_STR_IN}" = "" ]; then
        while [ 1 -eq 1 ]
        do
        red "You need to specify RTK_STR_IN or CABOT_SITE environment variable"
        snore 1
        done
        exit
    fi

    # launch str2str
    com="str2str -in ${RTK_STR_IN} -out serial://ttyUBLOX:${BAUD_UBLOX}"
    echo $com
    eval $com

# ros2 launch
else
    # if NTRIP_HOST exists in environment variables, use it.
    # if not, load NTRIP_HOST from CABOT_SITE package
    if [ "${NTRIP_HOST}" = "" ]; then
        echo "NTRIP_HOST does not exist in environment variables. load NTRIP_HOST from CABOT_SITE package"
        if [ "${CABOT_SITE}" != "" ]; then
        sitedir=`ros2 pkg prefix $CABOT_SITE`/share/$CABOT_SITE
            source $sitedir/config/rtk_config.sh
        fi
    fi

    # check if NTRIP_HOST is defined.
    if [ "${NTRIP_HOST}" = "" ]; then
        while [ 1 -eq 1 ]
        do
        red "You need to specify NTRIP_HOST or CABOT_SITE environment variable"
        snore 1
        done
    fi

    # ntrip client
    ntrip_client_arg=""
    if [ ${NTRIP_CLIENT_START_AT_LAUNCH} -eq 0 ]; then
        if [ "${NTRIP_CLIENT}" = "str2str_node" ]; then
            # launch str2str_node
            ntrip_client_arg="str2str_node:=true"
        elif [ "${NTRIP_CLIENT}" = "ntrip_client" ]; then
            # launch ntrip_client
            ntrip_client_arg="ntrip_client:=true"
        fi
    fi

    # gnss node
    gnss_arg=""
    if [ ${GNSS_NODE_START_AT_LAUNCH} -eq 0 ]; then
        gnss_arg="ublox_node:=true"
    fi

    com="ros2 launch mf_localization gnss.launch.py \
        $ntrip_client_arg \
        $gnss_arg \
        host:=$NTRIP_HOST \
        port:=$NTRIP_PORT \
        mountpoint:=$NTRIP_MOUNTPOINT \
        authentificate:=$NTRIP_AUTHENTIFICATE \
        username:=$NTRIP_USERNAME \
        password:=$NTRIP_PASSWORD \
        relay_back:=$NTRIP_STR2STR_RELAY_BACK \
        "
    echo $com
    eval $com
fi
