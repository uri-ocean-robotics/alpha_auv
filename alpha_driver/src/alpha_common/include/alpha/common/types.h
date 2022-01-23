#pragma once

#ifndef ALPHA_COMMON_TYPES_H
#define ALPHA_COMMON_TYPES_H

#include "cstdint"

#define STATIC_STRING static constexpr const char*
#define STATIC_DOUBLE static double

enum PwmMode : int {
    Undefined = -1,
    Thruster = 0,
    Pure =     1
};

typedef struct {
    float pressure;
    float depth;
    float temperature;
} pressure_t;

typedef struct {
    float voltage;
    float current;
    float power;
} multimeter_t;

typedef struct {
    int channel;
    float signal;
    uint8_t mode;
} pwm_t;


#endif