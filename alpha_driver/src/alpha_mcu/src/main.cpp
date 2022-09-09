#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"

#include "alpha/common.h"
#include "alpha/ina260.h"
#include "alpha/ms5837.h"
#include "alpha/handler.h"
#include "alpha/strobe.h"
#include "alpha/globals.h"

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