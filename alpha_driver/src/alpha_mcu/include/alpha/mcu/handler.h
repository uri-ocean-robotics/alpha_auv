#ifndef ALPHA_PARSER_H
#define ALPHA_PARSER_H

#include "cstdio"
#include "common.h"
#include "safety.h"
#include "pico/stdlib.h"
#include "iostream"
#include "cstring"
#include "pico/bootrom.h"

#include "nmea/nmea.h"
#include "alpha/common/dictionary.h"

[[noreturn]] void listen_incoming_messages();

namespace handler {

    /**
     * @brief Puts the system into usb boot state.
     */
    static inline void apply_usb_boot();

    /**
     * @brief Sets safety check state.
     * @param state if 1 overrides the safety subsystem. sets safety relay high.
     * @param state if 0 overrides the safety subsystem. sets safety relay low.
     * @param state if -1 lifts the override in safety subsystem.
     */
    static inline bool apply_safety(int state);

    /**
     * @brief Controls pwm channels
     * @param channel
     * @param signal
     */
    static inline bool apply_pwm_input(int channel, float signal);

    /**
     * @brief Initializes a channel with given mode.
     *
     * @param channel   Channel of the pwm signal
     * @param mode      Mode of the pwm channel. @see #PwmMode
     * @return true     If succeeds
     * @return false    If fails
     */
    static inline bool apply_pwm_enable(int channel, int mode);

}

#endif