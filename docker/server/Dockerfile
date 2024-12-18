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

FROM ubuntu:focal

ARG TZ="Etc/UTC"

ENV TZ=$TZ \
	DEBIAN_FRONTEND="noninteractive"

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | tee /etc/timezone

RUN apt update && \
	apt install -y \
	wget \
	curl \
	openjdk-11-jre \
	openjdk-11-jdk \
	zip \
	maven \
	git \
	curl \
	jq


ENV USERNAME runner_user

ARG UID=1001
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

WORKDIR /home/$USERNAME

# this often does not work due to server issue
RUN wget https://public.dhe.ibm.com/ibmdl/export/pub/software/openliberty/runtime/release/23.0.0.8/openliberty-javaee8-23.0.0.8.zip && \
	unzip openliberty-javaee8-23.0.0.8.zip && \
	cd wlp/bin && \
	./server create defaultServer
#
# use this if the server is not reachable, sometimes you can download the file from the browser, but not with the build script
# COPY openliberty-javaee8-19.0.0.12.zip openliberty-javaee8-19.0.0.12.zip
# RUN    unzip openliberty-javaee8-19.0.0.12.zip && \
#        cd wlp/bin && \
#        ./server create defaultServer
#

#RUN git clone https://github.com/daisukes/MapService -b hokoukukan_2018-docker && \
RUN git clone https://github.com/CMU-cabot/MapService -b cabot-hokoukukan_2018 && \
	cd MapService && \
	sh download-lib.sh

RUN cd MapService/MapService && \
	mvn initialize && \
	mvn package && \
        cp target/MapService-0.0.1-SNAPSHOT.war /home/$USERNAME/wlp/usr/servers/defaultServer/apps

# RUN git clone https://github.com/CMU-cabot/QueryService -b hokoukukan-2018 && \
RUN git clone https://github.com/CMU-cabot/QueryService -b kayukawa/search_by_category && \
	cd QueryService && \
	sh download-lib.sh

RUN cd QueryService/QueryService && \
	mvn initialize && \
	mvn package && \
        cp target/QueryService-0.0.1-SNAPSHOT.war /home/$USERNAME/wlp/usr/servers/defaultServer/apps

COPY server.xml /home/$USERNAME/wlp/usr/servers/defaultServer/
COPY server-data.sh /home/$USERNAME
COPY server-init.sh /home/$USERNAME

WORKDIR /home/$USERNAME/wlp/bin

