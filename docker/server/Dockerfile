# Copyright (c) 2022, 2025  Carnegie Mellon University and Miraikan
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

FROM ubuntu:jammy AS base

ARG TZ="Etc/UTC"
ARG UID=1001
ARG GID=1001

ENV TZ=$TZ \
        DEBIAN_FRONTEND="noninteractive"

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | tee /etc/timezone

RUN apt update && \
        apt install -y --no-install-recommends \
        curl \
        gosu \
        jq \
        openjdk-11-jre \
        zip \
        && \
        apt clean && \
        rm -rf /var/lib/apt/lists/*

ENV USERNAME runner_user

RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d/ && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid $UID $USERNAME && \
        groupmod --gid $UID $USERNAME

FROM base AS build

USER root
RUN apt update && \
        apt install -y --no-install-recommends \
        git \
        maven \
        openjdk-11-jdk \
        unzip \
        wget

USER $USERNAME
WORKDIR /home/$USERNAME
# this often does not work due to server issue
RUN wget https://public.dhe.ibm.com/ibmdl/export/pub/software/openliberty/runtime/release/23.0.0.8/openliberty-javaee8-23.0.0.8.zip && \
        unzip openliberty-javaee8-23.0.0.8.zip

ADD https://api.github.com/repos/CMU-cabot/MapService/git/refs/heads/cabot-hokoukukan_2018 MapService-version.json
RUN git clone https://github.com/CMU-cabot/MapService -b cabot-hokoukukan_2018 && \
        cd MapService && \
        sh download-lib.sh

RUN cd MapService/MapService && \
        mvn initialize && \
        mvn package

ADD https://api.github.com/repos/CMU-cabot/QueryService/git/refs/heads/hokoukukan-2018 QueryService-version.json
RUN git clone https://github.com/CMU-cabot/QueryService -b hokoukukan-2018 && \
        cd QueryService && \
        sh download-lib.sh

RUN cd QueryService/QueryService && \
        mvn initialize && \
        mvn package

FROM base AS final

USER $USERNAME
COPY --from=build --chown=$USERNAME:$USERNAME /home/$USERNAME/wlp /home/$USERNAME/wlp
WORKDIR /home/$USERNAME/wlp/bin
RUN ./server create defaultServer

COPY --from=build /home/$USERNAME/MapService/MapService/target/MapService-0.0.1-SNAPSHOT.war /home/$USERNAME/wlp/usr/servers/defaultServer/apps
COPY --from=build /home/$USERNAME/QueryService/QueryService/target/QueryService-0.0.1-SNAPSHOT.war /home/$USERNAME/wlp/usr/servers/defaultServer/apps
COPY --chown=$USERNAME:$USERNAME server.xml /home/$USERNAME/wlp/usr/servers/defaultServer/
COPY --chown=$USERNAME:$USERNAME server-data.sh /home/$USERNAME
COPY --chown=$USERNAME:$USERNAME server-init.sh /home/$USERNAME


USER root
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["./server", "run", "defaultServer"]
