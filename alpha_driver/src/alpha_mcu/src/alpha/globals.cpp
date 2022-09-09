#include "globals.h"
#include "memory"

namespace globals {
    multimeter_t multimeter_data = multimeter_t();

    pressure_t pressure_data = pressure_t();

    std::shared_ptr<INA260> multimeter = std::make_shared<INA260>();

    std::shared_ptr<MS5837> barometer = std::make_shared<MS5837>();

    std::shared_ptr<PwmController> pwm_chan0 = std::make_shared<PwmController>(GLOB_PWM_CHANNEL_0_PIN, 0);
    std::shared_ptr<PwmController> pwm_chan1 = std::make_shared<PwmController>(GLOB_PWM_CHANNEL_1_PIN, 1);
    std::shared_ptr<PwmController> pwm_chan2 = std::make_shared<PwmController>(GLOB_PWM_CHANNEL_2_PIN, 2);
    std::shared_ptr<PwmController> pwm_chan3 = std::make_shared<PwmController>(GLOB_PWM_CHANNEL_3_PIN, 3);
    std::shared_ptr<PwmController> pwm_chan4 = std::make_shared<PwmController>(GLOB_PWM_CHANNEL_4_PIN, 4);

    std::shared_ptr<Serial> a_uart0 = std::make_shared<Serial>(0, 115200);

    std::shared_ptr<Serial> a_uart1 = std::make_shared<Serial>(1, 115200);

    std::shared_ptr<Safety> safety = std::make_shared<Safety>();

    std::shared_ptr<Strobe> strobe = std::make_shared<Strobe>();

    void initialize() {

        pwm_chan0->initialize();
        pwm_chan1->initialize();
        pwm_chan2->initialize();
        pwm_chan3->initialize();
        pwm_chan4->initialize();

        globals::multimeter->initialize();

        globals::barometer->initialize();

        globals::safety->initialize();

        globals::strobe->initialize();

        globals::a_uart0->initialize();

        globals::a_uart1->initialize();

    }
}