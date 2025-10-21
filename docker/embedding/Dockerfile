ARG TARGETARCH

# AMD64
FROM --platform=linux/amd64 pytorch/pytorch:2.2.0-cuda11.8-cudnn8-runtime as base_amd64

# ARM64
FROM --platform=linux/arm64 dustynv/l4t-pytorch:2.2-r35.4.1 as base_arm64

# update pip url
ENV INDEX_HOST=jetson-ai-lab.io
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp5/cu114
ENV PIP_TRUSTED_HOST=pypi.jetson-ai-lab.io
ENV PIP_EXTRA_INDEX_URL=https://pypi.org/simple

# Target arch
FROM base_${TARGETARCH} AS base

# install ubuntu packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gosu \
    sudo \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# install python packages
COPY requirements.txt requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

ENV USERNAME developer

### replace 1000 with your user/group id
ARG UID=1000
RUN useradd -m $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    mkdir -p /etc/sudoers.d/ && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod  --uid $UID $USERNAME && \
    groupmod --gid $UID $USERNAME
# add user to video group to fix NvRmMemInitNvmap error on Jetson
# https://forums.developer.nvidia.com/t/jetpack-5-0-dp-nvrmmeminitnvmap-failed-with-permission-denied/219804
RUN usermod -aG video $USERNAME

COPY entrypoint.sh /entrypoint.sh

ENV HOME /home/$USERNAME
WORKDIR $HOME

# set TORCH_HOME
ENV TORCH_HOME=$HOME/.cache/torch

COPY --from=cabot_src ./mf_app $HOME/mf_app

WORKDIR $HOME/mf_app
ENTRYPOINT ["/entrypoint.sh"]
CMD ["uvicorn", "server.app:app", "--host", "127.0.0.1", "--port", "8001"]
