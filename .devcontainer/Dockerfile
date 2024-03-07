FROM ubuntu:20.04
ENV DEBIAN_FRONTEND noninteractive

RUN apt update && apt install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    gdb \
    git \
    libboost-dev \
    libssl-dev \
    pkg-config \
    wget \
    unzip \
    vim \
    tmux && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME &&\
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME &&\
    chsh -s /bin/bash $USERNAME
