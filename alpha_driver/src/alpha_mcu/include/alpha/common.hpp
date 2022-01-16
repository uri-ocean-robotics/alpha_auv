#ifndef ALPHA_COMMON_H
#define ALPHA_COMMON_H

#include "alpha/all.hpp"
#include "hardware/i2c.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


void initialize_i2c() {
    i2c_init(ALPHA_I2C_DEFAULT, 100 * 1000);
    gpio_set_function(ALPHA_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ALPHA_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ALPHA_I2C_SDA_PIN);
    gpio_pull_up(ALPHA_I2C_SCL_PIN);
}

#endif