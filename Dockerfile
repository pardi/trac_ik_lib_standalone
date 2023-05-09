FROM ubuntu:latest
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y libeigen3-dev cmake g++ gcc libboost-all-dev libnlopt-dev \
libnlopt-cxx-dev liborocos-kdl-dev libtinyxml2-dev curl gnupg2 \
liburdfdom-dev liburdfdom-headers-dev libconsole-bridge-dev git -y

RUN mkdir setup_dep && cd setup_dep/ && git clone https://github.com/pardi/kdl_parser.git
RUN mkdir -p setup_dep/kdl_parser/kdl_parser/build && cd setup_dep/kdl_parser/kdl_parser/build && cmake .. && make && make install

COPY . /app
