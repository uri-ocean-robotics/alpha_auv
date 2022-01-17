#include "alpha/mcu/pwm_manager.h"


PwmManager::PwmManager(uint16_t pin) : m_controller(pin) {
    m_channel = pin;
}

bool PwmManager::f_safety_checker(struct repeating_timer *t) {
    auto _this = (PwmManager*)t->user_data;
    if(is_nil_time(_this->m_last_comm)) {
        return true;
    }

    if(absolute_time_diff_us(_this->m_last_comm, get_absolute_time()) > 2999999) {
        _this->set_pwm(0);
    }

    return true;
}
bool PwmManager::f_reporter(struct repeating_timer *t) {
    auto _this = (PwmManager*)t->user_data;
    NMEA* msg = new NMEA();
    msg->construct("%s,%d,%.4f", NMEA_PWM_REPORT,
                   _this->m_channel,
                   _this->m_controller.get_magnitude()
    );

    std::cout << msg->get_raw() << std::endl;
    delete msg;
    return true;
}

void PwmManager::initialize() {
    m_controller.initialize();

    add_repeating_timer_ms(100, f_safety_checker, this, &m_safety_checker_timer);

    add_repeating_timer_ms(REPORT_THRUSTER_PERIOD, f_reporter, this, &m_reporter_timer);

}

void PwmManager::set_pwm(float signal, bool rate_control) {
    m_last_comm = get_absolute_time();
    if (rate_control) {
        m_controller.change_magnitude_limited(signal);
    } else {
        m_controller.change_magnitude(signal);
    }
}
