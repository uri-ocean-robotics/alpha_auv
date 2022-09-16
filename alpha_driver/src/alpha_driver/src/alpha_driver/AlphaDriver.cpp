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

#include "AlphaDriver.h"
#include "iostream"


AlphaDriver::AlphaDriver() :
        m_io(),
        m_serial_port(m_io)
{

}

AlphaDriver::AlphaDriver(std::string port, int baud) :
        m_io(),
        m_serial_port(m_io),
        m_port(port),
        m_baud(baud),
        m_active(true)
{

   m_serial_port.open(port);
   m_serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud));

    m_serial_read_th = boost::thread(boost::bind(&AlphaDriver::f_serial_read_loop, this));
}

void AlphaDriver::initialize() {

}

std::string AlphaDriver::f_serial_read_line() {
    char buf[BUFSIZ];
    memset(buf, 0, BUFSIZ);
    char c;
    for(int i = 0 ; i < BUFSIZ; i++) {
        boost::asio::read(m_serial_port, boost::asio::buffer(&c, 1));
        switch (c) {
            case '\r':
                break;
            case '\n':
                return std::string(buf);
            default:
                buf[i] = c;
        }
    }
    return std::string(buf);
}

void AlphaDriver::f_serial_read_loop() {
    while (m_active) {
        auto line = f_serial_read_line();
        if(m_serial_callback) {
            m_serial_callback(line);
        }
    }
}

bool AlphaDriver::f_serial_send_line(std::string msg) {
    msg += "\r\n";
    return boost::asio::write(m_serial_port, boost::asio::buffer(msg.c_str(), msg.size())) == msg.size();
}

void AlphaDriver::send_raw(std::string nmea) {
    f_serial_send_line(nmea);
}

void AlphaDriver::cmd_pwm(int channel, double pwm) {
    NMEA msg;
    msg.construct(NMEA_FORMAT_PWM_CMD, NMEA_PWM_CMD, channel, pwm);
    f_serial_send_line(std::string(msg.get_raw()));
}

void AlphaDriver::init_pwm(int channel, int mode) {
    NMEA msg;
    msg.construct(NMEA_FORMAT_PWM_INIT, NMEA_PWM_INITIALIZE, channel, mode);
    f_serial_send_line(std::string(msg.get_raw()));
}

void AlphaDriver::configure_safety(int state) {
    NMEA msg;
    msg.construct(NMEA_FORMAT_SAFETY_CMD, NMEA_SAFETY_CMD, state);
    f_serial_send_line(std::string(msg.get_raw()));
}