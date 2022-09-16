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

#include "handler.h"
#include "globals.h"

[[noreturn]] void listen_incoming_messages() {
    while(true) {
        std::string t;
        std::cin >> t;

        NMEA msg(t.c_str());

        msg.parse();

        if(!msg.get_valid()) {
            continue;
        }

        if( strcmp(msg.get_cmd(), NMEA_BOOTSEL_CMD) == 0) {

            handler::apply_usb_boot();
        } else if (strcmp(msg.get_cmd(), NMEA_SAFETY_CMD) == 0) {

            if (msg.get_argc() != 1) {
                continue;
            }

            int value;
            sscanf(msg.get_data(),"%*[^,],%d", &value);

            handler::apply_safety(
                    static_cast<int>(value)
            );

        } else if (strcmp(msg.get_cmd(), NMEA_PWM_CMD) == 0) {
            if(msg.get_argc() != 2) {
                continue;
            }
            int channel;
            float signal;
            sscanf(msg.get_data(), "%*[^,],%d,%f", &channel, &signal);
            handler::apply_pwm_input(channel, signal);
        } else if (strcmp(msg.get_cmd(), NMEA_PWM_INITIALIZE) == 0) {

            if(msg.get_argc() != 2) {
                continue;
            }
            int channel;
            int mode;
            std::cout  << "Incoming: " << msg.get_data() << std::endl;
            sscanf(msg.get_data(), "%*[^,],%d,%d", &channel, &mode);
            handler::apply_pwm_enable(channel, mode);
        } else if (strcmp(msg.get_cmd(), NMEA_STROBE_CMD) == 0) {
            if(msg.get_argc() != 1) {
                continue;
            }
            int state;
            sscanf(msg.get_data(), "%*[^,]s,%d", &state);
            handler::apply_strobe(state);
        } else if (strcmp(msg.get_cmd(), NMEA_SERIAL0_CMD) == 0) {
            // relay it back
            std::string m(msg.get_data());
            handler::relay_serial0(
                m.substr(strlen(msg.get_cmd()) + 1,std::string::npos));
        } else if (strcmp(msg.get_cmd(), NMEA_SERIAL1_CMD) == 0) {
            // relay it back
            std::string m(msg.get_data());
            handler::relay_serial1(
                m.substr(strlen(msg.get_cmd()) + 1,std::string::npos));
        }
    }
}

void handler::apply_usb_boot() {
    reset_usb_boot(0x00, 0x00);
}

bool handler::apply_safety(int state) {
    if (state == 1) {
        globals::safety->override_relay(true, true);
    } else if (state == -1) {
        globals::safety->override_relay(true, false);
    } else {
        globals::safety->override_relay(false, false);
    }
    return true;
}

bool handler::apply_pwm_input(int channel, float signal) {
    switch (channel) {
        case 0:
            globals::pwm_chan0->set_pwm(signal);
            break;
        case 1:
            globals::pwm_chan1->set_pwm(signal);
            break;
        case 2:
            globals::pwm_chan2->set_pwm(signal);
            break;
        case 3:
            globals::pwm_chan3->set_pwm(signal);
            break;
        case 4:
            globals::pwm_chan4->set_pwm(signal);
            break;
        default:
            break;
    }
    return true;
}

void handler::apply_strobe(int state) {
    if(state == 1) {
        globals::strobe->enable();
    } else if (state == 0) {
        globals::strobe->disable();
    } else {
        // do something about it
    }
}

bool handler::apply_pwm_enable(int channel, int mode) {

    std::cout << "ENABLE: " << channel << " WITH MODE: " << mode << std::endl;

    switch (channel) {
        case 0:
            globals::pwm_chan0->set_mode(mode);
            globals::pwm_chan0->enable();
            break;
        case 1:
            globals::pwm_chan1->set_mode(mode);
            globals::pwm_chan1->enable();
            break;
        case 2:
            globals::pwm_chan2->set_mode(mode);
            globals::pwm_chan2->enable();
            break;
        case 3:
            globals::pwm_chan3->set_mode(mode);
            globals::pwm_chan3->enable();
            break;
        case 4:
            globals::pwm_chan4->set_mode(mode);
            globals::pwm_chan4->enable();
            break;
        default:
            return false;
    }
    return true;
}

void handler::relay_serial0(const std::string& msg) {
    NMEA m;
    m.construct("%s", msg.c_str());
    globals::a_uart0->put_string(std::string(m.get_raw()));
}

void handler::relay_serial1(const std::string& msg) {
    NMEA m;
    m.construct("%s", msg.c_str());
    globals::a_uart1->put_string(std::string(m.get_raw()));
}


