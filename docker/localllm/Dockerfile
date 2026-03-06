ARG TARGETARCH
ARG LOCAL_LLM_MODEL_ID=sbintuitions/sarashina2.2-vision-3b

# AMD64
FROM --platform=linux/amd64 pytorch/pytorch:2.2.0-cuda11.8-cudnn8-runtime as base_amd64

# ARM64
FROM --platform=linux/arm64 dustynv/l4t-pytorch:r36.2.0 as base_arm64

# update pip url
ENV INDEX_HOST=jetson-ai-lab.io
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp5/cu114
ENV PIP_TRUSTED_HOST=pypi.jetson-ai-lab.io
ENV PIP_EXTRA_INDEX_URL=https://pypi.org/simple

# Target arch
FROM base_${TARGETARCH} AS base
ARG LOCAL_LLM_MODEL_ID=sbintuitions/sarashina2.2-vision-3b

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
RUN python3 - <<'PY'
import importlib

required = [
    "torch",
    "torchvision",
    "PIL",
    "google.protobuf",
    "sentencepiece",
    "accelerate",
    "transformers",
]

for name in required:
    importlib.import_module(name)

import transformers

expected = "4.57.1"
if transformers.__version__ != expected:
    raise RuntimeError(f"transformers version mismatch: {transformers.__version__} != {expected}")

print("localllm dependencies verified:", transformers.__version__)
PY

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
ENV HF_HOME=/opt/huggingface
ENV HF_HUB_CACHE=$HF_HOME/hub
# Keep legacy transformers cache path aligned with HF hub cache.
# This avoids cache misses between build-time snapshot download and runtime loading.
ENV TRANSFORMERS_CACHE=$HF_HUB_CACHE
ENV LOCAL_LLM_MODEL_ID=${LOCAL_LLM_MODEL_ID}
ENV LOCAL_LLM_LOCAL_FILES_ONLY=true

# Pre-download the model into the image cache so runtime can work offline.
RUN mkdir -p "$HF_HOME" "$HF_HUB_CACHE" && \
    chown -R $USERNAME:$USERNAME "$HF_HOME" && \
    gosu $USERNAME python3 - <<'PY'
import os
from huggingface_hub import snapshot_download

model_id = os.environ.get("LOCAL_LLM_MODEL_ID", "sbintuitions/sarashina2.2-vision-3b")
token = os.environ.get("HUGGINGFACE_HUB_TOKEN") or None
cache_dir = os.environ.get("HF_HUB_CACHE")

path = snapshot_download(
    repo_id=model_id,
    token=token,
    cache_dir=cache_dir,
    local_files_only=False,
)
print(f"Downloaded model cache for {model_id} to {path}")
PY

COPY --from=cabot_src ./mf_app $HOME/mf_app

WORKDIR $HOME/mf_app
ENTRYPOINT ["/entrypoint.sh"]
CMD ["uvicorn", "server.local_llm_app:app", "--host", "127.0.0.1", "--port", "8002"]
