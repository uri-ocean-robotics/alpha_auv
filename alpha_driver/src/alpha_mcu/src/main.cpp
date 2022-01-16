#include "alpha/all.hpp"

pressure_t g_pressure_data;

multimeter_t g_multimeter_data;

thruster_t g_thruster_data;

ThrusterManager g_thruster_manager = ThrusterManager(ESC_MAIN_PIN, ESC_HORIZONTAL_PIN, ESC_VERTICAL_PIN);

INA260 g_multimeter = INA260();

MS5837 g_barometer = MS5837();

Safety g_safety = Safety();

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