#include "alpha/mcu/pwm_controller.h"


PwmController::PwmController() {
    m_pulse_width = PULSE_WIDTH_CTR; // microseconds
    m_freq = 50; // hertz

}

PwmController::PwmController(int pin) {
    m_pin = pin;
    m_pulse_width = PULSE_WIDTH_CTR;
    m_freq = 50;

}

bool PwmController::initialize() {
    if(m_pin == -1) {
        return false;
    }
    add_repeating_timer_ms(m_limiter_period, f_limiter, this, &m_limiter_timer);
    gpio_set_function(m_pin, GPIO_FUNC_PWM);

    m_slice_num = pwm_gpio_to_slice_num(m_pin);

    uint32_t f_sys = clock_get_hz(clk_sys);

    float divider = f_sys / 1000000UL;

    pwm_set_clkdiv(m_slice_num, divider);

    m_top = 1000000UL / m_freq - 1;

    pwm_set_wrap(m_slice_num, m_top);

    pwm_set_chan_level(m_slice_num, 0, m_pulse_width);

    pwm_set_enabled(m_slice_num, true);

    return true;
}

void PwmController::f_change_pulse(uint16_t pulse) {
    if(pulse > PULSE_WIDTH_MAX) {
        pulse = PULSE_WIDTH_MAX;
    } else if (pulse < PULSE_WIDTH_MIN) {
        pulse = PULSE_WIDTH_MIN;
    } else {
        // There is no way
    }

    m_pulse_width = pulse;
    pwm_set_chan_level(m_slice_num, 0, m_pulse_width);
}

void PwmController::set_pin(uint16_t pin) { m_pin = pin; }

void PwmController::change_magnitude(double magnitude) {
    m_current = magnitude;
    if(magnitude < -1) {
        magnitude = 1;
    } else if (magnitude > 1) {
        magnitude = 1;
    }

    f_change_pulse(floor(magnitude * ((PULSE_WIDTH_MAX - PULSE_WIDTH_MIN) / 2.0) + PULSE_WIDTH_CTR));
}

void PwmController::change_magnitude_limited(double magnitude) {
    m_desired = magnitude;
}

double PwmController::get_magnitude() const {
    return (m_pulse_width - PULSE_WIDTH_CTR) / ((PULSE_WIDTH_MAX - PULSE_WIDTH_MIN) / 2.0);
}

double PwmController::get_pulse() {
    return m_pulse_width;
}

bool PwmController::f_limiter(struct repeating_timer *t) {
    auto _this = (PwmController*)t->user_data;

    double diff = _this->m_desired - _this->m_current;
    if(diff == 0) {
        return true;
    }

    double dmdt = diff / (_this->m_limiter_period / 1000.0);

    if(fabs(dmdt) > 5 /* slope */ ) {
        _this->m_current += sgn(diff) * 0.01; // increment
    } else {
        _this->m_current = _this->m_desired;
    }

    _this->change_magnitude(_this->m_current);
    return true;
}