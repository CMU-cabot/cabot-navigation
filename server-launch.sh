#!/bin/bash

# Copyright (c) 2022, 2024  Carnegie Mellon University and Miraikan
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

trap ctrl_c INT TERM KILL

function backup_mapdata() {
    log_name="MapData-$(date +%Y-%m-%d-%H-%M-%S).geojson"
    docker compose run --rm --no-deps map_data /home/runner_user/server-data.sh -e /cabot_site_pkg/.tmp/$log_name
    blue "data saved"
}

function ctrl_c() {
    blue "exit script is hooked"
    backup_mapdata

    cd $scriptdir
    docker compose -p $launch_prefix --profile $profile down
    exit
}

function err {
    >&2 red "[ERROR] "$@
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

function help()
{
    echo "Usage:"
    echo "-h           show this help"
    echo "-d           use cabot_sites instead of cabot_site_pkg"
    echo "-p <package> cabot site package name"
    echo "-f           ignore errors"
    echo "-v           verbose"
    echo "-c           clean the map_server before launch if the server is for different map"
    echo "-C           forcely clean the map_server"
    echo "-l           location tools server"
    echo "-E 1-10      separate environment (ROS_DOMAIN_ID, CABOT_MAP_SERVER_HOST) for simultaneous launch"
}

if [[ -e .env ]]; then
    set -a
    source .env
    set +a
fi

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

data_dir=
development=0
ignore_error=0
verbose=0
clean_server=0
location_tools=0
environment=
launch_prefix=
: ${MAP_SERVER_PORT:=9090}
profile=map

while getopts "hdE:p:fvcCl" arg; do
    case $arg in
        h)
            help
            exit
        ;;
        d)
            development=1
        ;;
        E)
            environment=$OPTARG
        ;;
        p)
            CABOT_SITE=$OPTARG
        ;;
        f)
            ignore_error=1
        ;;
        v)
            verbose=1
        ;;
        c)
            clean_server=1
        ;;
        C)
            clean_server=2
        ;;
        l)
            profile=tools
            location_tools=1
        ;;
    esac
done
shift $((OPTIND-1))

if [[ $development -eq 1 ]]; then
    CABOT_SITE_PKG_DIR=$scriptdir/cabot_sites
else
    : ${CABOT_SITE_PKG_DIR:=$scriptdir/cabot_site_pkg}    # default cabot_site_pkg
fi
blue "finding $CABOT_SITE in $CABOT_SITE_PKG_DIR"
data_dir=$(find $CABOT_SITE_PKG_DIR -wholename "*/$CABOT_SITE/server_data" | head -1)
: ${CABOT_MAP_SERVER_ENV_FILE:=$data_dir/server.env}

## private variables
pids=()

temp_dir=$scriptdir/.tmp
mkdir -p $temp_dir

launch_prefix=$(basename $scriptdir)
if [[ -n $environment ]]; then
    MAP_SERVER_PORT=$(($MAP_SERVER_PORT+environment*10))
    launch_prefix="${launch_prefix}-env${environment}"
fi

# forcely clean and extit
if [[ $clean_server -eq 2 ]]; then
    blue "Clean servers"

    services="map_server map_data mongodb_ms"
    if [[ $location_tools -eq 1 ]]; then
        services="location_tools mongodb_lt"
    fi
    for service in $services; do
        if [[ ! -z $(docker ps -f "name=${launch_prefix}-$service" -q -a) ]]; then
            blue "stopping ${launch_prefix}-$service"
            if [[ $service == "map_server" ]]; then
                backup_mapdata
            fi
            docker ps -f "name=${launch_prefix}-$service-"
            docker ps -f "name=${launch_prefix}-$service" -q -a | xargs docker stop
            docker ps -f "name=${launch_prefix}-$service" -q -a | xargs docker container rm
        fi
    done
    exit 0
fi

if [[ $location_tools -eq 1 ]]; then
    docker compose -p $launch_prefix --profile $profile up -d
    exit 0
fi


function check_server() {
    server=http://localhost:${MAP_SERVER_PORT}/map

    # check if the server data is same with the specified data
    echo "curl $server/content-md5 --fail > ${temp_dir}/${launch_prefix}-content-md5 2> /dev/null"
    curl $server/content-md5 --fail > ${temp_dir}/${launch_prefix}-content-md5 2> /dev/null
    if [[ $? -ne 0 ]]; then
        blue "There is no server or servers may be launched by the old script."
        blue "Servers will be cleaned"
        return 1
    else
        cd $data_dir
        pwd
        md5sum=$(find . -type f ! -name 'content-md5' ! -name 'attachments.zip' -exec md5sum {} + | LC_COLLATE=C sort -k 2 | md5sum)
        cd $scriptdir
        blue "md5sum - $md5sum"
        blue "server - $(cat ${temp_dir}/${launch_prefix}-content-md5)"
        if [[ $(cat ${temp_dir}/${launch_prefix}-content-md5) == $md5sum ]]; then  ## match, so no clean
            blue "md5 matched, do not relaunch server"
            return 0
        fi
    fi
    return 2
}

if [ ! -e $data_dir ]; then
    err "You should specify correct server data directory or cabot site package name"
    help
    exit 1
fi
blue "using $data_dir for map_server data"

if [[ $clean_server -eq 1 ]]; then
    if [ -z $data_dir ]; then
        err "You should specify correct server data directory or cabot site package name"
        help
        exit 1
    fi

    if check_server; then
        exit 0
    else
        blue "Clean servers"
        for service in "map_server" "map_data" "mongodb_ms"; do
            if [[ ! -z $(docker ps -f "name=${launch_prefix}-$service" -q -a) ]]; then
                if [[ $service == "map_server" ]]; then
                    backup_mapdata
                fi
                docker ps -f "name=${launch_prefix}-$service" -q -a | xargs docker stop
                docker ps -f "name=${launch_prefix}-$service" -q -a | xargs docker container rm
            fi
        done
    fi
else
    flag=0
    if check_server; then
        exit 0
    fi

    for service in "map_server" "map_data" "mongodb_ms"; do
        if [[ $(docker ps -f "name=${launch_prefix}-$service" -q | wc -l) -ne 0 ]]; then
            err "There is ${launch_prefix}-$service server running"
            flag=1
        fi
    done
    if [[ $flag -eq 1 ]]; then
        red "Please stop the servers with '-C' or '-c' option to clean before launch"
        exit 1
    fi
fi

## check data file
error=0
files="server.env MapData.geojson"
for file in $files; do
    if [ ! -e $data_dir/$file ]; then
        err "$data_dir/$file file does not exist";
        error=1;
    fi
done
# launch docker compose
if [ ! -e $data_dir/server.env ]; then
    error=1
    err "$data_dir/server.env file does not exist"
fi

if [ $error -eq 1 ] && [ $ignore_error -eq 0 ]; then
    err "add -f option to ignore file errors"
    exit 2
fi


export MAP_SERVER_PORT
export CABOT_SITE
export CABOT_SITE_PKG_DIR
if [ -e $CABOT_MAP_SERVER_ENV_FILE ]; then
    export CABOT_MAP_SERVER_ENV_FILE
fi
if [[ $verbose -eq 1 ]]; then
    docker compose -p $launch_prefix --profile $profile up -d
    docker compose --ansi never -p $launch_prefix logs -f
else
    docker compose -p $launch_prefix --profile $profile up -d
fi
