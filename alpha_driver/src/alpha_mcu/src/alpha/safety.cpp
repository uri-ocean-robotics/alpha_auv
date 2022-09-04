#include "alpha/common/dictionary.h"

#include "ina260.h"
#include "ms5837.h"
#include "safety.h"
#include "globals.h"
#include "iostream"


Safety::Safety() {

}

bool Safety::f_monitor_voltage(struct repeating_timer *t) {
    Safety* _this = (Safety*)t->user_data;

    bool old = _this->m_deadman_time_marked;

    if(globals::multimeter_data.voltage < SAFETY_DEADMAN_VOLTAGE) {
        _this->m_deadman_time_marked = true;
    } else {
        _this->m_deadman_time_marked = false;
    }

    if(old != _this->m_deadman_time_marked) {
        _this->m_deadman_time_mark = get_absolute_time();
    }

    if(_this->m_deadman_time_marked) {
        if(time_reached(delayed_by_ms(_this->m_deadman_time_mark, SAFETY_MAX_DURATION_DEADMAN_VOLTAGE_MS))) {
            _this->deactivate_relay();
        }
    }

    return true;
}

bool Safety::f_monitor_underwater_time(struct repeating_timer* t) {
    Safety* _this = (Safety*)t->user_data;

    bool old = _this->m_depth_time_marked;

    if(globals::pressure_data.depth > 1) {
        _this->m_depth_time_marked = true;
    } else {
        _this->m_depth_time_marked = false;
    }

    if(old != _this->m_depth_time_marked) {
        _this->m_depth_time_mark = get_absolute_time();
    }


    if(_this->m_depth_time_marked) {
        if(time_reached(delayed_by_ms(_this->m_depth_time_mark, SAFETY_MAX_DURATION_UNDERWATER_MS))) {
            _this->deactivate_relay();
        }
    }

    return true;
}

bool Safety::f_reporter(struct repeating_timer* t) {
    Safety* _this = (Safety*)t->user_data;

    NMEA* msg = new NMEA();
    msg->construct(NMEA_FORMAT_SAFETY_REPORT, NMEA_SAFETY_REPORT, _this->is_activated());
    std::cout << msg->get_raw() << std::endl;
    delete msg;

    return true;

}

void Safety::initialize() {
    m_override = false;
    gpio_init(DROP_WEIGHT_PIN);
    gpio_set_dir(DROP_WEIGHT_PIN, GPIO_OUT);

    m_depth_time_marked = false;
    m_deadman_time_marked = false;

    add_repeating_timer_ms(250, Safety::f_monitor_voltage, this, &m_voltage_monitor_timer);
    add_repeating_timer_ms(250, Safety::f_monitor_underwater_time, this, &m_depth_monitor_timer);
    add_repeating_timer_ms(REPORT_SAFETY_PERIOD, Safety::f_reporter, this, &m_reporter_timer);

    activate_relay();
}

void Safety::override_relay(bool override, bool state) {
    m_override = override;
    gpio_put(DROP_WEIGHT_PIN, state);
}

void Safety::activate_relay() const {
    if(!m_override) {
        gpio_put(DROP_WEIGHT_PIN, 1);
    }
}

void Safety::deactivate_relay() const {
    if(!m_override) {
        gpio_put(DROP_WEIGHT_PIN, 0);
    }
}

bool Safety::is_activated() {
    return gpio_get(DROP_WEIGHT_PIN);
}
