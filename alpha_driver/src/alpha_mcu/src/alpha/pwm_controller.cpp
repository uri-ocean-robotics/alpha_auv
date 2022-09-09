#include "pwm_controller.h"

#include <cmath>


PwmController::PwmController(int pin, int channel, int mode) {
    m_pin = pin;

    m_channel = channel;

    m_freq = 50;

    m_pulse_width = PULSE_WIDTH_CTR;

    m_mode = mode;

    m_current = 0;

    m_desired = 0;

    m_is_enabled = false;
}

void PwmController::initialize() {

    add_repeating_timer_ms(REPORT_PWM_PERIOD, f_reporter, this, &m_reporter_timer);

    add_repeating_timer_ms(m_limiter_period, f_limiter, this, &m_limiter_timer);

    gpio_set_function(m_pin, GPIO_FUNC_PWM);

    m_slice_num = pwm_gpio_to_slice_num(m_pin);

    uint32_t f_sys = clock_get_hz(clk_sys);

    float divider = f_sys / 1000000UL;

    pwm_set_clkdiv(m_slice_num, divider);

    m_top = 1000000UL / m_freq - 1;

    pwm_set_wrap(m_slice_num, m_top);

    // pwm_set_chan_level(m_slice_num, 0, m_pulse_width);

    // pwm_set_enabled(m_slice_num, true);

}
bool PwmController::f_reporter(struct repeating_timer *t) {

    auto self = (PwmController *) t->user_data;

    if(!self->m_is_enabled) {
        return true;
    }

    NMEA *msg = new NMEA();
    msg->construct(NMEA_FORMAT_PWM_REPORT,
                   NMEA_PWM_REPORT,
                   self->m_channel,
                   self->m_current,
                   self->m_mode,
                   self->m_is_enabled
    );

    std::cout << msg->get_raw() << std::endl;
    delete msg;
    return true;
}

void PwmController::set_pwm(float signal) {

    m_last_comm = get_absolute_time();

    if (m_mode == PwmMode::Thruster) {
        f_change_magnitude_limited(signal);
    } else {
        f_change_magnitude(signal);
    }
}

void PwmController::f_change_pulse(uint16_t pulse) {

    m_pulse_width = pulse;

    if(m_is_enabled) {
        pwm_set_chan_level(m_slice_num, 0, m_pulse_width);
    }
}

void PwmController::f_change_magnitude(float magnitude) {
    if(m_mode == PwmMode::Thruster) {
        magnitude = magnitude < -1 ? -1 : magnitude;
        magnitude = magnitude > 1 ? 1 : magnitude;

        m_current = magnitude;
        f_change_pulse(static_cast<uint16_t>
            (std::round(magnitude * ((PULSE_WIDTH_MAX - PULSE_WIDTH_MIN) / 2.0) + (PULSE_WIDTH_MAX + PULSE_WIDTH_MIN) / 2.0))
        );
    } else if (m_mode == PwmMode::Pure) {
        magnitude = magnitude < 0 ? 0 : magnitude;
        magnitude = magnitude > 1 ? 1 : magnitude;

        m_current = magnitude;
        f_change_pulse(static_cast<uint16_t>
            (std::round(magnitude * (PULSE_WIDTH_MAX - PULSE_WIDTH_MIN) + PULSE_WIDTH_MIN))
        );

    }
}

void PwmController::f_change_magnitude_limited(float magnitude) {
    m_desired = magnitude;
}

bool PwmController::f_limiter(struct repeating_timer *t) {
    auto self = (PwmController*)t->user_data;

    if(self->m_mode == PwmMode::Pure) {
        return false;
    }

    if(!self->m_is_enabled) {
        return true;
    }

    auto diff = self->m_desired - self->m_current;
    if(diff != 0) {

        float dmdt = diff / (static_cast<float>(self->m_limiter_period) / 1000.0f);

        if (std::fabs(dmdt) > 5 /* slope */ ) {
            self->m_current = self->m_current + sgn(diff) * 0.01f; // increment
        } else {
            self->m_current = self->m_desired;
        }

        self->f_change_magnitude(self->m_current);
    }

    return true;
}

void PwmController::enable() {

    m_is_enabled = true;

    m_pulse_width = m_mode == PwmMode::Pure ? PULSE_WIDTH_MIN : PULSE_WIDTH_CTR;

    pwm_set_chan_level(m_slice_num, 0, m_pulse_width);

    pwm_set_enabled(m_slice_num, true);



    add_repeating_timer_ms(100, f_safety_checker, this, &m_safety_checker_timer);

}

void PwmController::disable() {
    m_is_enabled = false;
    pwm_set_enabled(m_slice_num, false);
}

void PwmController::set_mode(int mode) {
    m_mode = mode;
}

bool PwmController::f_safety_checker(struct repeating_timer *t) {
    auto self = (PwmController*)t->user_data;

    if(is_nil_time(self->m_last_comm)) {
        return true;
    }

    if(absolute_time_diff_us(self->m_last_comm, get_absolute_time()) > 2999999) {
        self->set_pwm(0);
    }

    return true;
}