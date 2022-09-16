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

#ifndef ALPHA_RF_TELEOP_RF_COMMS_H
#define ALPHA_RF_TELEOP_RF_COMMS_H

#include "boost/asio/serial_port.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"

class RfComms {
private:
    boost::asio::io_service m_io;
    boost::asio::serial_port m_serial_port;
    boost::thread m_serial_read_th;
    std::string m_port;
    int m_baud;
    void f_serial_read_loop();
    std::string f_serial_read_line();
    boost::function <void(std::string)> m_serial_callback;
    bool m_active;

public:
    RfComms();
    RfComms(std::string port, int baud);

    bool activate();
    bool deactivate();

    bool sendLine(std::string msg);

    void setCallback(boost::function<void(std::string)> func);
};


#endif //ALPHA_RF_TELEOP_RF_COMMS_H
