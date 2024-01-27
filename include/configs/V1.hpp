#pragma once
#include <numbers>

#include "../Leg.hpp"
#include "../SSC32U.hpp"
#include "../Servo.hpp"

using std::numbers::pi;

constexpr float chassisWidth = 0.07183850f;
constexpr float chassisLength = 0.31559532f;
constexpr float upperLegLength = 0.192f;
constexpr float lowerLegLength = 0.192f;
constexpr float shoulderWidth = 0.04715084f;
constexpr float footRadius = 0.020f;

SSC32U pwmDriver{"/dev/ttyUSB0", B115200};

// Angle of leg fully scrunched: 0.82510782
Servo LFS{18, pwmDriver, 550, 1200, -pi / 2.0, 0};
Servo LFU{17, pwmDriver, 900, 1560, 0, pi / 2.0};
Servo LFL{16, pwmDriver, 1500, 1050, pi / 2.0, 0.82510782};

Servo RFS{29, pwmDriver, 1200, 530, -pi / 2.0, 0};
Servo RFU{30, pwmDriver, 2000, 1350, 0, pi / 2.0};
Servo RFL{31, pwmDriver, 1500, 2000, pi / 2.0, 0.82510782};

Servo LBS{2, pwmDriver, 1250, 600, -pi / 2.0, 0};
Servo LBU{1, pwmDriver, 500, 1200, 0, pi / 2.0};
Servo LBL{0, pwmDriver, 1500, 950, pi / 2.0, 0.82510782};

Servo RBS{13, pwmDriver, 1530, 2180, -pi / 2.0, 0};
Servo RBU{14, pwmDriver, 1230, 550, 0, pi / 2.0};
Servo RBL{15, pwmDriver, 1500, 2050, pi / 2.0, 0.82510782};

Leg LF{LFS,
       LFU,
       LFL,
       {chassisLength / 2.0, -chassisWidth / 2.0, 0},
       upperLegLength,
       lowerLegLength,
       footRadius,
       shoulderWidth};
Leg RF{RFS,
       RFU,
       RFL,
       {chassisLength / 2.0, chassisWidth / 2.0, 0},
       upperLegLength,
       lowerLegLength,
       footRadius,
       shoulderWidth};
Leg LB{LBS,
       LBU,
       LBL,
       {-chassisLength / 2.0, -chassisWidth / 2.0, 0},
       upperLegLength,
       lowerLegLength,
       footRadius,
       shoulderWidth};
Leg RB{RBS,
       RBU,
       RBL,
       {-chassisLength / 2.0, chassisWidth / 2.0, 0},
       upperLegLength,
       lowerLegLength,
       footRadius,
       shoulderWidth};