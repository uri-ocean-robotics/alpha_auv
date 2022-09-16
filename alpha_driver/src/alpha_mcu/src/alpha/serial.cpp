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

#include "serial.h"

std::string in0 = std::string();
std::string in1 = std::string();

Serial::Serial(int chan, int baud) {

    if(chan == 0) {
        m_hw = uart0;
    } else if (chan == 1) {
        m_hw = uart1;
    }

    m_baud = baud;

}


void uart_callback(uart_inst_t* u, std::string* in) {
    // read it into std::string
    while (uart_is_readable(u)) {
        char c = uart_getc(u);
        if(c == '\n') {

            NMEA *msg_in = new NMEA();
            NMEA *msg = new NMEA();

            msg_in->parse(in->c_str());
            if(msg_in->get_valid()) {

                msg->construct("%s,%s",
                               u == uart0 ? NMEA_SERIAL0_CMD : NMEA_SERIAL1_CMD,
                               msg_in->get_data());

                std::cout << msg->get_raw() << std::endl;
            } else {
                msg->construct("%s,invalid",
                               u == uart0 ? NMEA_SERIAL0_CMD : NMEA_SERIAL1_CMD);
                std::cout << msg->get_raw() << std::endl;
            }

            delete msg;
            delete msg_in;

            in->clear();
        } else {
            in->push_back(c);
        }
    }


}

void uart0_callback() {
    uart_callback(uart0, &in0);
}

void uart1_callback() {
    uart_callback(uart1, &in1);
}

void Serial::initialize() {

    int uart_irq;
    int rx_pin;
    int tx_pin;
    irq_handler_t irq;

    if(m_hw == uart0) {
        uart_irq = UART0_IRQ;
        rx_pin = 0;
        tx_pin = 1;
        irq = &uart0_callback;
    } else if (m_hw == uart1) {
        uart_irq = UART1_IRQ;
        rx_pin = 8;
        tx_pin = 9;
        irq = &uart1_callback;
    }

    uart_init(m_hw, m_baud);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    irq_set_exclusive_handler(uart_irq, irq);
    irq_set_enabled(uart_irq, true);
    uart_set_irq_enables(m_hw, true, false);

}

void Serial::put_string(const std::string &s) {

    uart_puts(m_hw, s.c_str());
    uart_puts(m_hw, "\r\n");

}