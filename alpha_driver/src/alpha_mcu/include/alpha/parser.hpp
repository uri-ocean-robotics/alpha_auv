#ifndef ALPHA_PARSER_H
#define ALPHA_PARSER_H

#include "cstdio"
#include "alpha/common.hpp"
#include "alpha/thrusters.hpp"
#include "alpha/safety.hpp"
#include "pico/stdlib.h"
#include "iostream"
#include "alpha/nmea.hpp"
#include "cstring"
#include "pico/bootrom.h"

void listen_incoming_messages() {
    auto incoming = new NMEA();
    while(true) {
        std::cin >> incoming->get_raw();

        incoming->parse();
        if( strcmp(incoming->get_cmd(), NMEA_BOOTSEL_CMD) == 0) {
            reset_usb_boot(0x00, 0x00);
        } else if (strcmp(incoming->get_cmd(), NMEA_THRUST_PWM_CMD) == 0) {
            if(incoming->get_argc() != 3) {
                continue;
            }
            g_thruster_manager.set_thrust(
                incoming->get_values()[NMEA_THRUST_PWM_MAIN_INDEX],
                incoming->get_values()[NMEA_THRUST_PWM_VERTICAL_INDEX],
                incoming->get_values()[NMEA_THRUST_PWM_HORIZONTAL_INDEX]
            );
        } else if (strcmp(incoming->get_cmd(), NMEA_SAFETY_CMD) == 0) {
            if(incoming->get_argc() != 1) {
                continue;
            }
            if(incoming->get_values()[0] == 1) {
                g_safety.override_relay(true, true);
            } else if (incoming->get_values()[0] == -1) {
                g_safety.override_relay(true, false);
            } else {
                g_safety.override_relay(false, false);
            }
        }

        memset((void*)incoming->get_raw(), 0x00, BUFSIZ);
    }
    delete incoming;
}

#endif