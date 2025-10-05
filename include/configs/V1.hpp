#pragma once
#include <numbers>

#include "../Chassis.hpp"
#include "../SSC32U.hpp"

using std::numbers::pi;

constexpr float chassisWidth = 0.07183850f;
constexpr float chassisLength = 0.31559532f;
constexpr float upperLegLength = 0.192f;
constexpr float lowerLegLength = 0.192f;
constexpr float shoulderWidth = 0.04715084f;
constexpr float footRadius = 0.020f;

SSC32U pwmDriver{"/dev/ttyUSB0", B115200};

// Angle of leg fully scrunched: 0.82510782
// Angle of leg fully extended: 2.82095426
Servo LFS{18, pwmDriver, 550, 1200, -pi / 2.0, 0};
Servo LFU{17, pwmDriver, 950, 1610, pi / 2.0, 0};
Servo LFL{16, pwmDriver, 1800, 1050, 0.82510782, 2.82095426};

Servo RFS{29, pwmDriver, 1200, 530, -pi / 2.0, 0};
Servo RFU{30, pwmDriver, 2100, 1450, pi / 2.0, 0};
Servo RFL{31, pwmDriver, 1200, 2050, 0.82510782, 2.82095426};

Servo LBS{2, pwmDriver, 1250, 600, -pi / 2.0, 0};
Servo LBU{1, pwmDriver, 500, 1200, pi / 2.0, 0};
Servo LBL{0, pwmDriver, 1750, 950, 0.82510782, 2.82095426};

Servo RBS{13, pwmDriver, 1530, 2180, -pi / 2.0, 0};
Servo RBU{14, pwmDriver, 1230, 550, pi / 2.0, 0};
Servo RBL{15, pwmDriver, 1200, 2050, 0.82510782, 2.82095426};

Chassis chassis{chassisWidth,
                chassisLength,
                upperLegLength,
                lowerLegLength,
                shoulderWidth,
                footRadius,
                LFS,
                LFU,
                LFL,
                RFS,
                RFU,
                RFL,
                LBS,
                LBU,
                LBL,
                RBS,
                RBU,
                RBL};