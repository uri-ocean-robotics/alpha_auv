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

#ifndef DROP_WEIGHT_HPP
#define DROP_WEIGHT_HPP

#include "nmea/nmea.h"
#include "dictionary.h"
#include "hardware/gpio.h"
#include "pico/time.h"


class Safety {
private:

    int m_relay_pin;

    struct repeating_timer m_voltage_monitor_timer;

    struct repeating_timer m_depth_monitor_timer;

    struct repeating_timer m_reporter_timer;

    absolute_time_t m_deadman_time_mark;

    absolute_time_t m_depth_time_mark;

    bool m_deadman_time_marked;

    bool m_depth_time_marked;

    bool m_override;

    static bool f_monitor_voltage(struct repeating_timer *t);

    static bool f_monitor_underwater_time(struct repeating_timer* t);

    static bool f_reporter(struct repeating_timer* t);

public:

    Safety();

    void initialize();

    void override_relay(bool override, bool state);

    void activate_relay() const;

    void deactivate_relay() const;

    static bool is_activated();

};


#endif