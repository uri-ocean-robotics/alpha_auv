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

#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"

#include "alpha/common.h"
#include "alpha/ina260.h"
#include "alpha/ms5837.h"
#include "alpha/handler.h"
#include "alpha/strobe.h"
#include "alpha/globals.h"

int main() {

    stdio_init_all();

    initialize_i2c();

    globals::initialize();

    multicore_launch_core1(listen_incoming_messages);

    // todo: is it needed?
    while(true) {
        sleep_ms(100);
    }

    return 0;
}