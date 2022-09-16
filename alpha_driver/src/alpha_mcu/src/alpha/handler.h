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

#ifndef ALPHA_PARSER_H
#define ALPHA_PARSER_H

#include "cstdio"
#include "common.h"
#include "safety.h"
#include "pico/stdlib.h"
#include "iostream"
#include "cstring"
#include "pico/bootrom.h"

#include "nmea/nmea.h"
#include "alpha/common/dictionary.h"

[[noreturn]] void listen_incoming_messages();

namespace handler {

    /**
     * @brief Puts the system into usb boot state.
     */
    static inline void apply_usb_boot();

    /**
     * @brief Sets safety check state.
     * @param state if 1 overrides the safety subsystem. sets safety relay high.
     * @param state if 0 overrides the safety subsystem. sets safety relay low.
     * @param state if -1 lifts the override in safety subsystem.
     */
    static inline bool apply_safety(int state);

    /**
     * @brief Controls pwm channels
     * @param channel
     * @param signal
     */
    static inline bool apply_pwm_input(int channel, float signal);

    /**
     * @brief Initializes a channel with given mode.
     *
     * @param channel   Channel of the pwm signal
     * @param mode      Mode of the pwm channel. @see #PwmMode
     * @return true     If succeeds
     * @return false    If fails
     */
    static inline bool apply_pwm_enable(int channel, int mode);

    /**
     * @brief Applies strobe command
     *
     * @param state 1 to turn on, 0 to turn off
     */
    static inline void apply_strobe(int state);

    static inline void relay_serial0(const std::string& msg);

    static inline void relay_serial1(const std::string& msg);

}

#endif