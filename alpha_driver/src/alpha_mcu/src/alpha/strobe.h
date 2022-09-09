#ifndef STROBE_HPP
#define STROBE_HPP

#include "nmea/nmea.h"
#include "dictionary.h"
#include "hardware/gpio.h"
#include "pico/time.h"


class Strobe {
private:

    int m_strobe_pin;

    struct repeating_timer m_reporter_timer;

    struct repeating_timer m_strobe_flashing_timer;

    bool m_is_active;

    static bool f_flash(struct repeating_timer *t);

    static bool f_reporter(struct repeating_timer* t);

public:

    Strobe();

    void initialize();

    void enable() const;

    void disable() const;

    static bool is_activated();

};


#endif