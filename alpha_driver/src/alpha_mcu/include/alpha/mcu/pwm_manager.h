#ifndef ALPHA_MCU_PWM_MANAGER_H
#define ALPHA_MCU_PWM_MANAGER_H

#include "alpha/common/dictionary.h"
#include "cmath"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "common.h"
#include "dictionary.h"
#include "iostream"

#include "nmea/nmea.h"
#include "pwm_controller.h"

class PwmManager {
private:

    static const uint16_t PIN_UNASSIGNED = 0xff;

    PwmController m_controller;

    uint16_t m_channel;

    absolute_time_t m_last_comm;

    struct repeating_timer m_safety_checker_timer;

    struct repeating_timer m_reporter_timer;

    static bool f_safety_checker(struct repeating_timer *t);

    static bool f_reporter(struct repeating_timer *t);

public:

    PwmManager(uint16_t pin = PIN_UNASSIGNED);

    void initialize();

    void set_pwm(float signal, bool rate_control = true);


};


#endif //ALPHA_MCU_PWM_MANAGER_H
