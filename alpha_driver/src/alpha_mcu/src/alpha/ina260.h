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

#ifndef ALPHA_INA260_H
#define ALPHA_INA260_H

#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "dictionary.h"
#include "nmea/nmea.h"
#include "pico/time.h"
#include "alpha/common/dictionary.h"

// Declerations
class INA260 {

private:
    int m_i2c_addr;

    struct repeating_timer m_reporter_timer;

    int m_alert_pin;

    static bool f_reporter(struct repeating_timer *t);

    static void f_alert_callback(uint , uint32_t);

public:

    INA260(int addr_ = 0x40, int alert_pin = MULTIMETER_ALERT_PIN);

    void initialize();

    bool is_exist() const;

    int raw_read(uint8_t pointer_addr, unsigned short *val_) const;

    int raw_write(char pointer_addr, unsigned short val_) const;

    int get_voltage(double *V_) const;

    int get_current(double *I_) const;

    int get_power(double *P_) const;

    int set_config(unsigned short val) const;

    int set_alert(unsigned short val) const;

    void read_alert() const;

    int set_limit(unsigned short val) const;


};

#endif