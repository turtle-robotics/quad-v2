# TURTLE QUAD V2
[![GitHub Org's stars](https://img.shields.io/github/stars/turtle-robotics?style=flat&logo=github&label=TURTLE%20Robotics&color=gold)](https://github.com/turtle-robotics)

This repository contains the source code for the second revision of TURTLE's quadruped project (QUAD).

Dependencies:
 - Eigen 3
 - yaml-cpp
 - mjbots moteus
 - mjbots pi3hat

CMake command (run once to configure):
```sh
mkdir -p build
cd build
cmake .. --preset rpi
```

Make command (run after cmake to build):
```sh
make
```