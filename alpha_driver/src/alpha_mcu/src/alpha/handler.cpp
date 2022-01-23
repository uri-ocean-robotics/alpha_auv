#include "alpha/mcu/handler.h"
#include "alpha/mcu/globals.h"

[[noreturn]] void listen_incoming_messages() {
    while(true) {
        std::string t;
        std::cin >> t;

        NMEA msg(t.c_str());

        msg.parse();
        if( strcmp(msg.get_cmd(), NMEA_BOOTSEL_CMD) == 0) {
            handler::apply_usb_boot();
        } else if (strcmp(msg.get_cmd(), NMEA_SAFETY_CMD) == 0) {
            if (msg.get_argc() != 1) {
                continue;
            }
            handler::apply_safety(
                    static_cast<int>(msg.get_values()[0])
            );
        } else if (strcmp(msg.get_cmd(), NMEA_PWM_CMD) == 0) {
            if(msg.get_argc() != 2) {
                continue;
            }
            handler::apply_pwm_input(
                    static_cast<int>(msg.get_values()[0]),
                    static_cast<float>(msg.get_values()[1])
            );
        } else if (strcmp(msg.get_cmd(), NMEA_PWM_INITIALIZE) == 0) {
            if(msg.get_argc() != 2) {
                continue;
            }
            handler::apply_pwm_enable(
                    static_cast<int>(msg.get_values()[0]),
                    static_cast<int>(msg.get_values()[1])
            );
        }
    }
}

void handler::apply_usb_boot() {
    reset_usb_boot(0x00, 0x00);
}

bool handler::apply_safety(int state) {
    if (state == 1) {
        globals::safety.override_relay(true, true);
    } else if (state == -1) {
        globals::safety.override_relay(true, false);
    } else {
        globals::safety.override_relay(false, false);
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

bool handler::apply_pwm_enable(int channel, int mode) {
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