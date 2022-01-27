#include "alpha/mcu/common.h"
#include "alpha/mcu/dictionary.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

void initialize_i2c() {
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(ALPHA_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ALPHA_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ALPHA_I2C_SDA_PIN);
    gpio_pull_up(ALPHA_I2C_SCL_PIN);
}