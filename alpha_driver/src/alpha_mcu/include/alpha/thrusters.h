#ifndef ALPHA_THRUSTERS_H
#define ALPHA_THRUSTERS_H

#include "alpha_common/dictionary.h"
#include "cmath"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#include "alpha/common.h"
// #include "alpha/nmea.hpp"
#include "alpha/dictionary.h"
#include "iostream"

#include "nmea/nmea.h"
#include "alpha/types.h"

// Declerations
class ThrusterManager;
class PwmController;

// Definitions
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

    void change_pulse(uint16_t pulse);

    uint16_t m_limiter_period = 4; // ms

public:
    PwmController();

    PwmController(int pin);

    bool init();

    void set_pin(uint16_t pin);

    void change_magnitude(double magnitude);

    void change_magnitude_limited(double magnitude);

    double get_magnitude() const;

    double get_pulse();

    static bool limiter(struct repeating_timer *t);

};

class PwmManager {
private:

    static const uint16_t PIN_UNASSIGNED = 0xff;

    PwmController m_controller;

    uint16_t m_channel;

    absolute_time_t m_last_comm;

    struct repeating_timer m_safety_checker_timer;

    struct repeating_timer m_reporter_timer;

    static bool safety_checker(struct repeating_timer *t);

    static bool report_thrusters(struct repeating_timer *t);

public:

    PwmManager(uint16_t pin = PIN_UNASSIGNED);

    void initialize();

    void set_pwm(float signal, bool rate_control = true);


};

class ThrusterManager{
    private:
        PwmController m_main;
        PwmController m_horizontal;
        PwmController m_vertical;

        absolute_time_t m_last_comm;

        struct repeating_timer m_safety_checker_timer;

        struct repeating_timer m_reporter_timer;

        static bool safety_checker(struct repeating_timer *t);

        static bool report_thrusters(struct repeating_timer *t);

    public:
        ThrusterManager(int pin_main, int pin_horizontal, int pin_vertical);

        void initialize();

        void set_thrust(float main, float vertical, float horizontal);


};

extern ThrusterManager g_thruster_manager;

extern PwmManager g_pwm_channels[];

#endif