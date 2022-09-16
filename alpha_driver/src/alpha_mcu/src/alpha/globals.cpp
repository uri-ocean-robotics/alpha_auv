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

#include "globals.h"
#include "memory"

namespace globals {
    multimeter_t multimeter_data = multimeter_t();

    pressure_t pressure_data = pressure_t();

    INA260* multimeter = new INA260();

    MS5837* barometer = new MS5837();

    PwmController* pwm_chan0 = new PwmController(GLOB_PWM_CHANNEL_0_PIN, 0);
    PwmController* pwm_chan1 = new PwmController(GLOB_PWM_CHANNEL_1_PIN, 1);
    PwmController* pwm_chan2 = new PwmController(GLOB_PWM_CHANNEL_2_PIN, 2);
    PwmController* pwm_chan3 = new PwmController(GLOB_PWM_CHANNEL_3_PIN, 3);
    PwmController* pwm_chan4 = new PwmController(GLOB_PWM_CHANNEL_4_PIN, 4);

    Serial* a_uart0 = new Serial(0, 115200);

    Serial* a_uart1 = new Serial(1, 115200);

    Safety* safety = new Safety();

    Strobe* strobe = new Strobe();

    void initialize() {

        pwm_chan0->initialize();
        pwm_chan1->initialize();
        pwm_chan2->initialize();
        pwm_chan3->initialize();
        pwm_chan4->initialize();

        globals::multimeter->initialize();

        globals::barometer->initialize();

        globals::safety->initialize();

        globals::strobe->initialize();

        globals::a_uart0->initialize();

        globals::a_uart1->initialize();

    }
}