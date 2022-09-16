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

#ifndef ALPHA_MCU_GLOBALS_H
#define ALPHA_MCU_GLOBALS_H

// STL
#include "map"
#include "array"
#include "memory"

// Project
#include "alpha/common/types.h"
#include "ms5837.h"
#include "ina260.h"
#include "pwm_controller.h"
#include "safety.h"
#include "strobe.h"
#include "serial.h"

namespace globals {
    extern multimeter_t multimeter_data;

    extern pressure_t pressure_data;

    extern INA260* multimeter;

    extern MS5837* barometer;

    extern PwmController* pwm_chan0;

    extern PwmController* pwm_chan1;

    extern PwmController* pwm_chan2;

    extern PwmController* pwm_chan3;

    extern PwmController* pwm_chan4;

    extern Safety* safety;

    extern Strobe* strobe;

    extern Serial* a_uart0;

    extern Serial* a_uart1;

    void initialize();
}


#endif //ALPHA_MCU_GLOBALS_H
