/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "common.h"
#include "dictionary.h"

void initialize_i2c() {
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(ALPHA_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ALPHA_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ALPHA_I2C_SDA_PIN);
    gpio_pull_up(ALPHA_I2C_SCL_PIN);
}