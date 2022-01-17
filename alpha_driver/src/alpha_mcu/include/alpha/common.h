#ifndef ALPHA_COMMON_H
#define ALPHA_COMMON_H

#include "alpha/all.hpp"
#include "hardware/i2c.h"

extern pressure_t g_pressure_data;

extern multimeter_t g_multimeter_data;

extern thruster_t g_thruster_data;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void initialize_i2c();

#endif