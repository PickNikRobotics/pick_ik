# syntax=docker/dockerfile:1.2
ARG ROS_DISTRO="rolling"
FROM ros:${ROS_DISTRO}-ros-base AS upstream
# Restate for later use
ARG ROS_DISTRO
RUN rm /ros_entrypoint.sh

# prevent interactive messages in apt install
ARG DEBIAN_FRONTEND=noninteractive

# install build dependencies
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    apt-get update && apt-get upgrade -y \
    && apt-get install -q -y --no-install-recommends \
    build-essential \
    ccache \
    clang-14 \
    clang-format-14 \
    clang-tidy \
    cmake \
    gdb \
    git \
    lld \
    ninja-build \
    openssh-client \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget \
    && rosdep update \
    && colcon mixin update default \
    && rm -rf /var/lib/apt/lists/*

# install pre-commit
RUN python3 -m pip install -U \
    pre-commit

# setup ssh
RUN mkdir /root/.ssh \
    && ssh-keyscan -t rsa github.com >> ~/.ssh/known_hosts

# build source dependencies
WORKDIR /opt/upstream
COPY upstream.repos .
RUN --mount=type=cache,target=/var/cache/apt,id=apt --mount=type=ssh \
    mkdir src \
    && vcs import src < upstream.repos \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && rosdep update && apt-get update \
    && rosdep install -q -y \
    --from-paths src \
    --ignore-src \
    --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*
RUN --mount=type=cache,target=/root/.ccache,id=ccache \
    . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build --mixin ccache rel-with-deb-info compile-commands lld \
    && ccache -s

# install dependencies of this project
WORKDIR /root/ws
COPY ../ ./src/pick_ik
# install repo dependencies
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    . /opt/upstream/install/setup.sh \
    && rosdep update && apt-get update \
    && rosdep install -q -y \
    --from-paths src \
    --ignore-src \
    --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*
RUN rm -rf /root/ws

FROM upstream AS development

ARG USER=dev
ARG UID=1000
ARG GID=$UID

# Setup user home directory
COPY ./.devcontainer/.profile /home/dev/.profile
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    groupadd --gid $GID $USER \
    && useradd --uid $GID --gid $UID -m $USER --groups sudo \
    && apt-get update \
    && echo $USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER \
    && chmod 0440 /etc/sudoers.d/$USER \
    && mkdir -p /home/${USER}/ws \
    && touch /home/${USER}/.bashrc \
    && chown -R ${GID}:${UID} /home/${USER}

USER $USER

FROM development as devcontainer

ENV SHELL /bin/bash
ENTRYPOINT []
CMD [ "sleep", "infinity" ]
