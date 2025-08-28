ARG TARGETARCH

# AMD64
FROM --platform=linux/amd64 pytorch/pytorch:2.3.1-cuda11.8-cudnn8-runtime as base_amd64

# ARM64
FROM --platform=linux/arm64 nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3 as base_arm64

# Target arch
FROM base_${TARGETARCH} AS base

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
USER $USERNAME

ENV HOME /home/$USERNAME
WORKDIR $HOME

COPY --from=cabot_src ./mf_app $HOME/mf_app

WORKDIR $HOME/mf_app
CMD ["uvicorn", "server.app:app", "--host", "127.0.0.1", "--port", "8001"]
