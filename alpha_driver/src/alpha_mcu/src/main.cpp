#include "alpha/all.hpp"




int main() {

    stdio_init_all();

    initialize_i2c();

    g_thruster_manager.initialize();

    g_multimeter.initialize();

    g_barometer.initialize();

    multicore_launch_core1(listen_incoming_messages);

    while(true) {
        sleep_ms(100);
    }

    return 0;
}