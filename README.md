<img src="banner.svg" alt="QUAD V2 Banner" style="width:100%;max-height:400px;"/>

[![GitHub Org's stars](https://img.shields.io/github/stars/turtle-robotics?style=flat&logo=github&label=TURTLE%20Robotics&color=gold)](https://github.com/turtle-robotics) ![Build](https://github.com/turtle-robotics/quad-v2/actions/workflows/build.yml/badge.svg)

Source code for the second revision of TURTLE's quadruped project (QUAD).

# Dependencies
Build requires CMake (with Make) and the following packages:
 - Debian/Ubuntu: `apt install libeigen3-dev libspdlog-dev libgtest-dev`
 - Arch: `pacman -S eigen spdlog gtest`

Building for Raspberry Pi requires an aarch64 compiler: `aarch64-linux-gnu-gcc`

Optionally Doxygen can be used to generate documentation for this project.

# Build
## Generate Buildsystem (once on setup)
```sh
cmake --fresh --preset rpi
```

> [!NOTE]
> There are multiple presets. Use preset `rpi` for building for the Raspberry Pi and `local` for building/testing locally (x86-64)

## Build the Project
To build:
```sh
make -C build-rpi
```

To build & deploy to the robot:
```sh
make -C build-rpi deploy
```