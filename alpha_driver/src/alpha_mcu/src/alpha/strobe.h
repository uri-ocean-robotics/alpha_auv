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

#ifndef STROBE_HPP
#define STROBE_HPP

#include "nmea/nmea.h"
#include "dictionary.h"
#include "hardware/gpio.h"
#include "pico/time.h"


class Strobe {
private:

    int m_strobe_pin;

    struct repeating_timer m_reporter_timer;

    struct repeating_timer m_strobe_flashing_timer;

    bool m_is_active;

    static bool f_flash(struct repeating_timer *t);

    static bool f_reporter(struct repeating_timer* t);

public:

    Strobe();

    void initialize();

    void enable() const;

    void disable() const;

    static bool is_activated();

};


#endif