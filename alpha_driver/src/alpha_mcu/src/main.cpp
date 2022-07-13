#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"

#include "alpha/mcu/common.h"
#include "alpha/mcu/ina260.h"
#include "alpha/mcu/ms5837.h"
#include "alpha/mcu/handler.h"
#include "alpha/mcu/strobe.h"
#include "alpha/mcu/globals.h"

int main() {

    stdio_init_all();

    initialize_i2c();

    globals::initialize();

    multicore_launch_core1(listen_incoming_messages);

    // todo: is it needed?
    while(true) {
        sleep_ms(100);
    }

    return 0;
}