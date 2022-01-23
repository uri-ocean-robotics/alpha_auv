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

    int m_mode;

    uint16_t m_top;

    int m_channel;

    absolute_time_t m_last_comm;

    int m_pin;

    float m_desired;

    float m_current;

    bool m_is_enabled;

    const uint16_t m_limiter_period = 4; // ms

    struct repeating_timer m_safety_checker_timer;

    struct repeating_timer m_reporter_timer;

    struct repeating_timer m_limiter_timer;

    void f_change_pulse(uint16_t pulse);

    static bool f_safety_checker(struct repeating_timer *t);

    static bool f_reporter(struct repeating_timer *t);

    static bool f_limiter(struct repeating_timer *t);

    void f_change_magnitude(float magnitude);

    void f_change_magnitude_limited(float magnitude);

public:

    explicit PwmController(int pin, int channel, int mode = PwmMode::Undefined);

    void initialize();

    void set_pwm(float signal);

    void enable();

    void disable();

    void set_mode(int mode);


};

#endif //ALPHA_MCU_PWM_CONTROLLER_H
