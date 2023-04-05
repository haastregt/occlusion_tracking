FROM ubuntu:20.04

RUN set -ex;                    \
    apt-get update;             \
    DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata; \
    apt-get install -y g++ cmake libboost-all-dev libgmp-dev libmpfr-dev libqt5opengl5-dev libqt5svg5-dev qtbase5-dev libeigen3-dev --no-install-recommends; \
    apt-get install -y python3-pip; \
    pip3 install pybind11[global]

WORKDIR /build