#include "alpha/mcu/globals.h"

namespace globals {
    multimeter_t multimeter_data = multimeter_t();

    pressure_t pressure_data = pressure_t();

    pwm_t pwm_chan0_data = pwm_t();
    pwm_t pwm_chan1_data = pwm_t();
    pwm_t pwm_chan2_data = pwm_t();
    pwm_t pwm_chan3_data = pwm_t();
    pwm_t pwm_chan4_data = pwm_t();


    pwm_t *pwm_chans_data[] = {
            &pwm_chan0_data,
            &pwm_chan1_data,
            &pwm_chan2_data
    };

    INA260   multimeter = INA260();
    MS5837   barometer = MS5837();
    PwmManager pwm_chan0_manager = PwmManager(GLOB_PWM_CHANNEL_0_PIN);
    PwmManager pwm_chan1_manager = PwmManager(GLOB_PWM_CHANNEL_1_PIN);
    PwmManager pwm_chan2_manager = PwmManager(GLOB_PWM_CHANNEL_2_PIN);

    PwmManager* pwm_managers[] = {
            &globals::pwm_chan0_manager,
            &globals::pwm_chan1_manager,
            &globals::pwm_chan2_manager
    };

    Safety safety = Safety();

    void initialize() {

        for(int i = 0 ; i < GLOB_PWM_CHANNEL_COUNT ; i++) {
            globals::pwm_managers[i]->initialize();
        }

        globals::multimeter.initialize();

        globals::barometer.initialize();

        globals::safety.initialize();

    }
}