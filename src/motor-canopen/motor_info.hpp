#pragma once
#include "identity.hpp"

struct MotorInfo{
    Identity id;
    float winding_r_ohm;
    uint8_t hall_offset;
};