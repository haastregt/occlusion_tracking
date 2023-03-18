FROM ubuntu:20.04

RUN set -ex;                    \
    apt-get update;             \
    DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata; \
    apt-get install -y g++ cmake libboost-all-dev libgmp-dev libmpfr-dev qt5-default --no-install-recommends; \
    apt-get install -y libcgal-dev libcgal-qt5-dev python3-pip; \
    pip3 install pybind11[global]

WORKDIR /build