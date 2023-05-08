FROM ubuntu:latest
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y libeigen3-dev cmake g++ gcc libboost-all-dev
RUN apt-get install -y libnlopt-dev liborocos-kdl-dev