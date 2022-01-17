#pragma once

#ifndef ALPHA_COMMON_TYPES_H
#define ALPHA_COMMON_TYPES_H

#include "cstdint"

#define STATIC_STRING static constexpr const char*
#define STATIC_DOUBLE static double

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
    float x;
    float y;
    float z;
} thruster_t;

typedef struct {
    uint8_t channel;
    float signal;
} pwm_t;

#endif