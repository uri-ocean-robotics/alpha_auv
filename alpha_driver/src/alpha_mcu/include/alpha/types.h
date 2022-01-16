#ifndef ALPHA_TYPES_H
#define ALPHA_TYPES_H


typedef struct pressure_t {
    float pressure;
    float depth;
    float temperature;
} pressure_t;

typedef struct multimeter_t {
    float voltage;
    float current;
    float power;
} multimeter_t;

typedef struct thruster_t {
    float x;
    float y;
    float z;
} thruster_t;

extern pressure_t g_pressure_data;

extern multimeter_t g_multimeter_data;

extern thruster_t g_thruster_data;

#endif