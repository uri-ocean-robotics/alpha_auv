#ifndef ALPHA_MCU_PWM_CONTROLLER_H
#define ALPHA_MCU_PWM_CONTROLLER_H

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

class PwmController {
private:
    uint16_t m_freq;

    uint32_t m_slice_num;

    uint16_t m_pulse_width;

    int m_pin = -1;

    uint16_t m_top;

    double m_desired;

    double m_current;

    struct repeating_timer m_limiter_timer;

    void f_change_pulse(uint16_t pulse);

    uint16_t m_limiter_period = 4; // ms

    static bool f_limiter(struct repeating_timer *t);

public:
    PwmController();

    PwmController(int pin);

    bool initialize();

    void set_pin(uint16_t pin);

    void change_magnitude(double magnitude);

    void change_magnitude_limited(double magnitude);

    double get_magnitude() const;

    double get_pulse();


};

#endif //ALPHA_MCU_PWM_CONTROLLER_H
