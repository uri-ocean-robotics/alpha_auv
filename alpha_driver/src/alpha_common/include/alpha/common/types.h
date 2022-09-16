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