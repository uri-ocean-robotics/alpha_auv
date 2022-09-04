#ifndef ALPHA_MCU_GLOBALS_H
#define ALPHA_MCU_GLOBALS_H

// STL
#include "map"
#include "array"
#include "memory"

// Project
#include "alpha/common/types.h"
#include "ms5837.h"
#include "ina260.h"
#include "pwm_controller.h"
#include "safety.h"
#include "strobe.h"
#include "serial.h"

namespace globals {
    extern multimeter_t multimeter_data;

    extern pressure_t pressure_data;

    extern std::shared_ptr<INA260> multimeter;

    extern std::shared_ptr<MS5837> barometer;

    extern std::shared_ptr<PwmController> pwm_chan0;

    extern std::shared_ptr<PwmController> pwm_chan1;

    extern std::shared_ptr<PwmController> pwm_chan2;

    extern std::shared_ptr<PwmController> pwm_chan3;

    extern std::shared_ptr<PwmController> pwm_chan4;

    extern std::shared_ptr<Safety> safety;

    extern std::shared_ptr<Strobe> strobe;

    extern std::shared_ptr<Serial> a_uart0;

    extern std::shared_ptr<Serial> a_uart1;

    void initialize();
}


#endif //ALPHA_MCU_GLOBALS_H
