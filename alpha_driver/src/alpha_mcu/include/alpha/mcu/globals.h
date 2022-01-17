#ifndef ALPHA_MCU_GLOBALS_H
#define ALPHA_MCU_GLOBALS_H

#include "alpha/common/types.h"

#include "ms5837.h"
#include "ina260.h"
#include "pwm_manager.h"
#include "safety.h"

void initialize_globals();

namespace globals {
    extern multimeter_t multimeter_data;

    extern pressure_t pressure_data;

    extern pwm_t pwm_chan0_data;

    extern pwm_t pwm_chan1_data;

    extern pwm_t pwm_chan2_data;

    extern pwm_t pwm_chan3_data;

    extern pwm_t pwm_chan4_data;

    extern pwm_t* pwm_chans_data[GLOB_PWM_CHANNEL_COUNT];

    extern INA260 multimeter;

    extern MS5837 barometer;

    extern PwmManager pwm_chan0_manager;

    extern PwmManager pwm_chan1_manager;

    extern PwmManager pwm_chan2_manager;

    extern PwmManager pwm_chan3_manager;

    extern PwmManager pwm_chan4_manager;

    extern PwmManager* pwm_managers[GLOB_PWM_CHANNEL_COUNT];

    extern Safety safety;

    void initialize();
}


#endif //ALPHA_MCU_GLOBALS_H
