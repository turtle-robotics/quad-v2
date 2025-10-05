#pragma once
#include "Chassis.hpp"

class Robot
{
public:
    Robot(Chassis chassis) : chassis{chassis} {};
    void walk();

private:
    Chassis chassis;
};

#ifdef QUAD_V1
#include "configs/V1.hpp"
#endif