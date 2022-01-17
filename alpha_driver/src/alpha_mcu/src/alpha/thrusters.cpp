#include "alpha/thrusters.h"


void PwmController::change_pulse(uint16_t pulse) {
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

PwmController::PwmController() {
    m_pulse_width = PULSE_WIDTH_CTR; // microseconds
    m_freq = 50; // hertz

    add_repeating_timer_ms(m_limiter_period, limiter, this, &m_limiter_timer);

}

PwmController::PwmController(int pin) {
    m_pin = pin;
    m_pulse_width = PULSE_WIDTH_CTR;
    m_freq = 50;
    add_repeating_timer_ms(m_limiter_period, limiter, this, &m_limiter_timer);
}

bool PwmController::init() {
    if(m_pin == -1) {
        return false;
    }

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

void PwmController::set_pin(uint16_t pin) { m_pin = pin; }

void PwmController::change_magnitude(double magnitude) {
    m_current = magnitude;
    if(magnitude < -1) {
        magnitude = 1;
    } else if (magnitude > 1) {
        magnitude = 1;
    }

    change_pulse(floor(magnitude * ((PULSE_WIDTH_MAX - PULSE_WIDTH_MIN) / 2.0) + PULSE_WIDTH_CTR));
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

bool PwmController::limiter(struct repeating_timer *t) {
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

bool PwmManager::safety_checker(struct repeating_timer *t) {
    auto _this = (PwmManager*)t->user_data;
    if(is_nil_time(_this->m_last_comm)) {
        return true;
    }

    if(absolute_time_diff_us(_this->m_last_comm, get_absolute_time()) > 2999999) {
        _this->set_pwm(0);
    }

    return true;
}
bool PwmManager::report_thrusters(struct repeating_timer *t) {
    auto _this = (PwmManager*)t->user_data;
    NMEA* msg = new NMEA();
    msg->construct("%s,%.3f,%.4f,%.4f", NMEA_THRUST_PWM_REPORT,
                   _this->m_channel,
                   _this->m_controller.get_magnitude()
    );

    std::cout << msg->get_raw() << std::endl;
    delete msg;
    return true;
}

PwmManager::PwmManager(uint16_t pin) : m_controller(pin) {
    m_channel = pin;
}

void PwmManager::initialize() {
    m_controller.init();

    add_repeating_timer_ms(100, safety_checker, this, &m_safety_checker_timer);
    add_repeating_timer_ms(REPORT_THRUSTER_PERIOD, report_thrusters, this, &m_reporter_timer);

}

void PwmManager::set_pwm(float signal, bool rate_control) {
    m_last_comm = get_absolute_time();
    if(rate_control) {
        m_controller.change_magnitude_limited(signal);
    } else {
        m_controller.change_magnitude(signal);
    }
}

bool ThrusterManager::safety_checker(struct repeating_timer *t) {
    auto _this = (ThrusterManager*)t->user_data;
    if(is_nil_time(_this->m_last_comm)) {
        return true;
    }

    if(absolute_time_diff_us(_this->m_last_comm, get_absolute_time()) > 2999999) {
        _this->set_thrust(0, 0, 0);
    }

    return true;
}

bool ThrusterManager::report_thrusters(struct repeating_timer *t) {
    auto _this = (ThrusterManager*)t->user_data;
    NMEA* msg = new NMEA();
    msg->construct("%s,%.3f,%.4f,%.4f", NMEA_THRUST_PWM_REPORT,
                   _this->m_main.get_magnitude(),
                   _this->m_vertical.get_magnitude(),
                   _this->m_horizontal.get_magnitude()
    );

    std::cout << msg->get_raw() << std::endl;
    delete msg;
    return true;
}

ThrusterManager::ThrusterManager(int pin_main, int pin_horizontal, int pin_vertical) :
        m_main(pin_main),
        m_horizontal(pin_horizontal),
        m_vertical(pin_vertical)
{

}

void ThrusterManager::initialize() {
    m_main.init();
    m_horizontal.init();
    m_vertical.init();


    add_repeating_timer_ms(100, safety_checker, this, &m_safety_checker_timer);
    add_repeating_timer_ms(REPORT_THRUSTER_PERIOD, report_thrusters, this, &m_reporter_timer);
}

void ThrusterManager::set_thrust(float main, float vertical, float horizontal) {
    m_last_comm = get_absolute_time();
    m_main.change_magnitude_limited(main);
    m_vertical.change_magnitude_limited(vertical);
    m_horizontal.change_magnitude_limited(horizontal);

    g_thruster_data.x = main;
    g_thruster_data.y = horizontal;
    g_thruster_data.z = vertical;

}

thruster_t g_thruster_data;

ThrusterManager g_thruster_manager = ThrusterManager(ESC_MAIN_PIN, ESC_HORIZONTAL_PIN, ESC_VERTICAL_PIN);

PwmManager g_pwm_channels[5] = {
        PwmManager(GLOB_PWM_CHANNEL_0_PIN),
        PwmManager(GLOB_PWM_CHANNEL_1_PIN),
        PwmManager(GLOB_PWM_CHANNEL_2_PIN),
        PwmManager(GLOB_PWM_CHANNEL_3_PIN),
        PwmManager(GLOB_PWM_CHANNEL_4_PIN),
};