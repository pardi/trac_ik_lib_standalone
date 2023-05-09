[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![build](https://github.com/pardi/trac_ik_lib_standalone/actions/workflows/docker-image.yml/badge.svg?event=push)

# TRAC-IK-Lib a standalone implementation

Trac-IK is an Inverse Kinematics library developed by [TracLabs](https://traclabs.com/). It is based on KDL library and implements a faster alternative Inverse Kinematics solver to the popular inverse Jacobian methods in KDL. Two algorithms are proposed:
- An extension to the KDL's Newton-based convergence algorithm, which detects and mitigates local minima due to joint limits by random jumps. 
- A SQP (Sequential Quadratic Programming) nonlinear optimization approach, which uses quasi-Newton methods that better handle joint limits.


# Why the fork?

The original implementation of Trac-IK is tightly linked with the ROS (Robotic Operating System) and do not allow an easy installation outside that environment. 

This repo adapts the current trac_ik_lib package to become a standlone library stripping down dependencies with ROS.

# Requirements
Install dependencies with:
```
sudo apt-get update && sudo apt-get install -y libeigen3-dev cmake g++ gcc libboost-all-dev libnlopt-dev \
libnlopt-cxx-dev liborocos-kdl-dev libtinyxml2-dev curl gnupg2 \
liburdfdom-dev liburdfdom-headers-dev libconsole-bridge-dev git
```

Install [kdl_parser library](https://github.com/pardi/kdl_parser.git) as follow:
```
git clone https://github.com/pardi/kdl_parser.git
mkdir -p kdl_parser/kdl_parser/build && cd setup_dep/kdl_parser/kdl_parser/build && cmake .. && make && make install
```
**N.B.** The above library is a fork of the original `kdl_parser` that fixes an installation issue when ROS is not involved.

# Build
```
git clone https://github.com/pardi/trac_ik_lib_standalone.git
mkdir -p trac_ik_lib_standalone/build
make -DCMAKE_BUILD_TYPE=[Debug | Release]
sudo make install
```

# TODO
- [x] Tag the first version of the standalone library
- [ ] Set up testing with different robots
- [ ] Improve error-handling


