/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#ifndef MS5837_H_BLUEROBOTICS
#define MS5837_H_BLUEROBOTICS

#include "hardware/i2c.h"
#include "nmea/nmea.h"
#include "dictionary.h"
#include "pico/time.h"
#include "alpha/common/types.h"


const uint8_t MS5837_ADDR = 0x76;
const uint8_t MS5837_RESET = 0x1E;
const uint8_t MS5837_ADC_READ = 0x00;
const uint8_t MS5837_PROM_READ = 0xA0;
const uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
const uint8_t MS5837_CONVERT_D2_8192 = 0x5A;

const uint8_t MS5837_02BA01 = 0x00; // Sensor version: From MS5837_02BA datasheet Version PROM Word 0
const uint8_t MS5837_02BA21 = 0x15; // Sensor version: From MS5837_02BA datasheet Version PROM Word 0
const uint8_t MS5837_30BA26 = 0x1A; // Sensor version: From MS5837_30BA datasheet Version PROM Word 0


class MS5837 {
private:

    uint16_t m_c[8];
    uint32_t m_D1_pres;
    uint32_t m_D2_temp;
    int32_t m_TEMP;
    int32_t m_P;
    uint8_t m_model;

    float m_fluid_density = 1023; // kg/m^3 (freshwater)

    struct repeating_timer m_reporter_timer;

    /** Performs calculations per the sensor data sheet for conversion and
     *  second order compensation.
     */
    void f_calculate();

    uint8_t f_crc4(uint16_t n_prom[]);

    static bool f_reporter(struct repeating_timer *t);

public:
    static constexpr float Pa = 100.f;
    static constexpr float bar = 0.001f;
    static constexpr float mbar = 1.0f;

    static const uint8_t MS5837_30BA = 0;
    static const uint8_t MS5837_02BA = 1;
    static const uint8_t MS5837_UNRECOGNISED = 255;

    MS5837();


    bool initialize();

    /** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA (default)
     * and MS5837::MS5837_02BA.
     */
    void set_model(uint8_t model);

    uint8_t get_model();

    /** Provide the density of the working fluid in kg/m^3. Default is for
     * seawater. Should be 997 for freshwater.
     */
    void set_fluid_density(float density);
    /** The read from I2C takes up to 40 ms, so use sparingly is possible.
     */
    void read();

    /** Pressure returned in mbar or mbar*conversion rate.
     */
    float pressure(float conversion = 1.0f);

    /** Temperature returned in deg C.
     */
    float temperature();

    /** Depth returned in meters (valid for operation in incompressible
     *  liquids only. Uses density that is set for fresh or seawater.
     */
    float depth();


};

#endif
