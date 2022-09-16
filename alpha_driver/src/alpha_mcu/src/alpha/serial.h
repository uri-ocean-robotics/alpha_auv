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

#ifndef ALPHA_MCU_SERIAL_H
#define ALPHA_MCU_SERIAL_H

// PICO
#include "hardware/uart.h"
#include "hardware/gpio.h"

// STL
#include "functional"
#include "memory"
#include "iostream"

// Project
#include "nmea/nmea.h"
#include "alpha/common/dictionary.h"


extern std::string in0;
extern std::string in1;

class Serial {
private:

    int m_baud;

    uart_inst_t * m_hw;

public:

    Serial(int chan, int baud);

    void initialize();

    void put_string(const std::string& s);

};


#endif
