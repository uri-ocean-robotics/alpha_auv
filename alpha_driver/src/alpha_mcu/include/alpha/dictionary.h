#ifndef ALPHA_DICTIONARY_H
#define ALPHA_DICTIONARY_H


#define NMEA_THRUST_PWM_CMD "PWMC"
#define NMEA_THRUST_PWM_REPORT "PWMR"
#define NMEA_BAROMETER_REPORT "BARR"
#define NMEA_MULTIMETER_REPORT "MULR"
#define NMEA_MULTIMETER_ALRT_REPORT "MLAR"
#define NMEA_BOOTSEL_CMD "BTSC"
#define NMEA_SAFETY_CMD "SAFC"
#define NMEA_SAFETY_REPORT "SAFR"

#define NMEA_THRUST_PWM_MAIN_INDEX 0
#define NMEA_THRUST_PWM_HORIZONTAL_INDEX 1
#define NMEA_THRUST_PWM_VERTICAL_INDEX 2

// TODO: redefine pin numbers
#define ESC_MAIN_PIN            2
#define ESC_HORIZONTAL_PIN      4
#define ESC_VERTICAL_PIN        6
#define PULSE_WIDTH_MAX 2000
#define PULSE_WIDTH_MIN 1000
#define PULSE_WIDTH_CTR 1500
#define REPORT_THRUSTER_PERIOD 150

#define REPORT_BAROMETER_PERIOD 200

#define REPORT_MULTIMETER_PERIOD 100
#define MULTIMETER_ALERT_PIN 31

#define ALPHA_I2C_DEFAULT i2c_default
#define ALPHA_I2C_SDA_PIN 16
#define ALPHA_I2C_SCL_PIN 17

#define DROP_WEIGHT_PIN 3
#define SAFETY_MAX_DURATION_UNDERWATER_MS 10 * 60 * 1000
#define SAFETY_MAX_DURATION_DEADMAN_VOLTAGE_MS 15 * 1000
#define SAFETY_DEADMAN_VOLTAGE 11
#define REPORT_SAFETY_PERIOD 1000

#endif