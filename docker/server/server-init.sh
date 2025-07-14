#!/bin/bash

# Copyright (c) 2022  Carnegie Mellon University
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

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

HOST=http://map_server:8080/map
data_dir=$(find /cabot_site_pkg -wholename "*/$CABOT_SITE/server_data" | head -1)
cp -r $data_dir .
admin=hulopadmin
pass=please+change+password
editor=editor

count=0
echo "waiting server is up"
while [ "$(curl -I $HOST/login.jsp 2>/dev/null | head -n 1 | cut -d' ' -f2)" != "200" ];
do
    snore 1
    UPLINE=$(tput cuu1)
    ERASELINE=$(tput el)
    echo -n "$UPLINE$ERASELINE"
    echo "waiting server is up ($count)"
    count=$((count+1))
done

blue "adding editor user"
curl -b admin-cookie.txt -c admin-cookie.txt $HOST/admin.jsp
curl -b admin-cookie.txt -c admin-cookie.txt -d "redirect_url=admin.jsp&user=${admin}&password=${pass}" $HOST/login.jsp
curl -b admin-cookie.txt -d "user=$editor&password=$editor&password2=$editor&role=editor&role=auditor" "$HOST/api/user?action=add-user"

blue "importing attachments.zip"
pushd server_data
mkdir -p attachments
find . -type f ! -name 'content-md5' ! -name 'attachments.zip' -exec md5sum {} + | LC_COLLATE=C sort -k 2 | md5sum > attachments/content-md5
pushd attachments
zip -r ../attachments.zip .
popd

if [ -e attachments.zip ]; then
    curl -b admin-cookie.txt -c admin-cookie.txt $HOST/admin.jsp
    curl -b admin-cookie.txt -c admin-cookie.txt -d "redirect_url=admin.jsp&user=${admin}&password=${pass}" $HOST/login.jsp
    curl -b admin-cookie.txt -F file=@attachments.zip "$HOST/api/admin?action=import&type=attachment.zip"
fi
popd

if [[ -e ./server_data/MapData.geojson ]]; then
    blue "importing MapData.geojson"
    ./server-data.sh -i ./server_data/MapData.geojson
fi
