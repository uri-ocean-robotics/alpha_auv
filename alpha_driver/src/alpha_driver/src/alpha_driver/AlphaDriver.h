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

#ifndef ALPHA_DRIVER_ALPHADRIVER_H
#define ALPHA_DRIVER_ALPHADRIVER_H

#include "boost/asio/serial_port.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"

#include "nmea/nmea.h"
#include "alpha/common/dictionary.h"


class AlphaDriver{
protected:

    boost::asio::io_service m_io;

    boost::asio::serial_port m_serial_port;

    boost::thread m_serial_read_th;

    std::string m_port;

    int m_baud;

    bool m_active;

    boost::function <void(std::string)> m_serial_callback;

    void f_serial_read_loop();

    bool f_serial_send_line(std::string msg);

    std::string f_serial_read_line();

public:
    AlphaDriver();

    enum SafetyState : int {
        OVERRIDE_OFF = -1,
        OVERRIDE_RELEASE = 0,
        OVERRIDE_ON = 1
    };

    AlphaDriver(std::string port, int baud);

    void initialize();

    void send_raw(std::string nmea);

    void set_port(decltype(m_port) val) { m_port = val;}
    auto get_port() -> decltype(m_port) {return m_port; }

    void set_baud(decltype(m_baud) val) { m_baud = val;}
    auto get_baud() -> decltype(m_baud) {return m_baud;}

    void set_serial_callback(decltype(m_serial_callback) c) { m_serial_callback  = c;}
    auto get_serial_callback() -> decltype(m_serial_callback) {return m_serial_callback;}

    void cmd_pwm(int channel, double pwm);

    void init_pwm(int channel, int mode);

    void configure_safety(int state);


};


#endif //ALPHA_DRIVER_ALPHADRIVER_H
