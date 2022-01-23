#ifndef ALPHA_MCU_GLOBALS_H
#define ALPHA_MCU_GLOBALS_H

#include "alpha/common/types.h"

#include "ms5837.h"
#include "ina260.h"
#include "pwm_controller.h"
#include "safety.h"
#include "memory"
#include "map"
#include "array"


namespace globals {
    extern multimeter_t multimeter_data;

    extern pressure_t pressure_data;

    extern INA260 multimeter;

    extern MS5837 barometer;

    extern std::shared_ptr<PwmController> pwm_chan0;

    extern std::shared_ptr<PwmController> pwm_chan1;

    extern std::shared_ptr<PwmController> pwm_chan2;

    extern std::shared_ptr<PwmController> pwm_chan3;

    extern std::shared_ptr<PwmController> pwm_chan4;

    extern Safety safety;

    void initialize();
}


#endif //ALPHA_MCU_GLOBALS_H
