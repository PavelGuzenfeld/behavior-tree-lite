FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        clang-18 clang-format-18 clang-tidy-18 cmake doctest-dev g++-14 git ninja-build \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

ENV CC=gcc-14 CXX=g++-14

WORKDIR /repo
