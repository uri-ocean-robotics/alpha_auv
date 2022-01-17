#include "alpha/mcu/execution.h"
#include "alpha/mcu/globals.h"

[[noreturn]] void listen_incoming_messages() {
    auto incoming = new NMEA();
    while(true) {
        std::cin >> incoming->get_raw();

        incoming->parse();
        if( strcmp(incoming->get_cmd(), NMEA_BOOTSEL_CMD) == 0) {
            comm::apply_usb_boot();
        } else if (strcmp(incoming->get_cmd(), NMEA_SAFETY_CMD) == 0) {
            if (incoming->get_argc() != 1) {
                continue;
            }
            comm::apply_safety(
                    (int)incoming->get_values()[0]
            );
        } else if (strcmp(incoming->get_cmd(), NMEA_PWM_CMD) == 0) {
            if(incoming->get_argc() != 2) {
                continue;
            }
            comm::apply_pwm(
                    (int) incoming->get_values()[0],
                    (float) incoming->get_values()[1],
                    (bool) (incoming->get_values()[2] == 0)
            );
        }

        memset((void*)incoming->get_raw(), 0x00, BUFSIZ);
    }
    delete incoming;
}

void comm::apply_usb_boot() {
    reset_usb_boot(0x00, 0x00);
}

void comm::apply_safety(int state) {
    if (state == 1) {
        globals::safety.override_relay(true, true);
    } else if (state == -1) {
        globals::safety.override_relay(true, false);
    } else {
        globals::safety.override_relay(false, false);
    }
}

void comm::apply_pwm(int channel, float signal, bool rate_control) {
    if(channel > 5) {
        return;
    }

    if(channel < 0) {
        return;
    }

    globals::pwm_managers[channel]->set_pwm(signal, rate_control);
}

