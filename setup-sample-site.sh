#!/bin/bash

git clone \
    https://github.com/CMU-cabot/cabot_sites_cmu \
    -b ros2 \
    cabot_sites/cabot_sites_cmu

if ! which jq; then
    echo "you need jq command, install by apt install jq"
    exit
fi

url=$(curl https://api.github.com/repos/cmu-cabot/cabot_sites_cmu/releases/latest | jq -r .assets[0].browser_download_url -)

if [[ -n $url ]]; then
    cd cabot_site_pkg
    curl -L $url -o cabot_site_cmu_3d.zip
    unzip cabot_site_cmu_3d.zip
fi
