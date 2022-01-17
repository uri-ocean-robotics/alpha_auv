#ifndef DROP_WEIGHT_HPP
#define DROP_WEIGHT_HPP

#include "nmea/nmea.h"
#include "alpha/dictionary.h"
#include "hardware/gpio.h"
#include "pico/time.h"


class Safety {
private:

    int m_relay_pin;

    struct repeating_timer m_voltage_monitor_timer;

    struct repeating_timer m_depth_monitor_timer;

    struct repeating_timer m_reporter_timer;

    absolute_time_t m_deadman_time_mark;

    absolute_time_t m_depth_time_mark;

    bool m_deadman_time_marked;

    bool m_depth_time_marked;

    bool m_override;

    static bool f_monitor_voltage(struct repeating_timer *t);

    static bool f_monitor_underwater_time(struct repeating_timer* t);

    static bool f_reporter(struct repeating_timer* t);

public:

    Safety();

    void override_relay(bool override, bool state);

    void activate_relay();

    void deactivate_relay();

    bool is_activated();

};


extern Safety g_safety;

#endif