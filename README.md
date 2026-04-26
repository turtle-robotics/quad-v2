<img src="banner.svg" alt="QUAD V2 Banner" style="width:100%;max-height:400px;"/>

[![GitHub Org's stars](https://img.shields.io/github/stars/turtle-robotics?style=flat&logo=github&label=TURTLE%20Robotics&color=gold)](https://github.com/turtle-robotics)

Source code for the second revision of TURTLE's quadruped project (QUAD).

# Dependencies
Build requires CMake (with ninja) and the following packages:
 - Debian/Ubuntu: `apt install libeigen3-dev libspdlog-dev libgtest-dev`
 - Arch: `pacman -S eigen spdlog gtest`

Optionally Doxygen can be used to generate documentation for this project.

# Build
## Generate Buildsystem (once on setup)
```sh
cmake -B build --fresh --preset rpi
```

> [!NOTE]
> There are multiple presets. Use preset `rpi` for building for the Raspberry Pi and `local` for building/testing locally (x86-64)

## Build the Project
To build:
```sh
make -C build
```

To build & deploy to the robot:
```sh
make -C build deploy
```