#include "alpha/ms5837.h"
#include "iostream"
#include "alpha_common/dictionary.h"

MS5837 g_barometer = MS5837();

pressure_t g_pressure_data;

MS5837::MS5837() {

}

void MS5837::calculate() {
    // Given C1-C6 and D1, D2, calculated TEMP and P
    // Do conversion first and then second order temp compensation

    int32_t dT = 0;
    int64_t SENS = 0;
    int64_t OFF = 0;
    int32_t SENSi = 0;
    int32_t OFFi = 0;
    int32_t Ti = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    // Terms called
    dT = D2_temp-uint32_t(C[5])*256l;
    if ( _model == MS5837_02BA ) {
        SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l;
        OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l;
        P = (D1_pres*SENS/(2097152l)-OFF)/(32768l);
    } else {
        SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
        OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
        P = (D1_pres*SENS/(2097152l)-OFF)/(8192l);
    }

    // Temp conversion
    TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;

    //Second order compensation
    if ( _model == MS5837_02BA ) {
        if((TEMP/100)<20){         //Low temp
            Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
            OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
            SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
        }
    } else {
        if((TEMP/100)<20){         //Low temp
            Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
            OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
            SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
            if((TEMP/100)<-15){    //Very low temp
                OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
                SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
            }
        }
        else if((TEMP/100)>=20){    //High temp
            Ti = 2*(dT*dT)/(137438953472LL);
            OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
            SENSi = 0;
        }
    }

    OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
    SENS2 = SENS-SENSi;

    TEMP = (TEMP-Ti);

    if ( _model == MS5837_02BA ) {
        P = (((D1_pres*SENS2)/2097152l-OFF2)/32768l);
    } else {
        P = (((D1_pres*SENS2)/2097152l-OFF2)/8192l);
    }
}

uint8_t MS5837::crc4(uint16_t n_prom[]) {
    uint16_t n_rem = 0;

    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;

    for ( uint8_t i = 0 ; i < 16; i++ ) {
        if ( i%2 == 1 ) {
            n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
        } else {
            n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
        }
        for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
            if ( n_rem & 0x8000 ) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    n_rem = ((n_rem >> 12) & 0x000F);

    return n_rem ^ 0x00;
}

bool MS5837::initialize() {

    i2c_write_blocking(ALPHA_I2C_DEFAULT, MS5837_ADDR, &MS5837_RESET, 1, false);
    // Wait for reset to complete
    busy_wait_ms(10);

    // Read calibration values and CRC
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t m = MS5837_PROM_READ + i * 2;
        uint8_t dest[2];
        if(i2c_write_blocking(ALPHA_I2C_DEFAULT, MS5837_ADDR, &m, 1, true) != PICO_ERROR_GENERIC) {
            i2c_read_blocking(ALPHA_I2C_DEFAULT, MS5837_ADDR, dest, 2, false);
            C[i] = static_cast<unsigned short>(dest[0]);
            C[i] = (C[i] << 8) | static_cast<unsigned short>(dest[1]);
        }
    }

    // Verify that data is correct with CRC
    uint8_t crcRead = C[0] >> 12;
    uint8_t crcCalculated = crc4(C);

    if (crcCalculated != crcRead) {
        return false; // CRC fail
    }

    uint8_t version = (C[0] >> 5) & 0x7F; // Extract the sensor version from PROM Word 0

    // Set _model according to the sensor version
    if (version == MS5837_02BA01) {
        _model = MS5837_02BA;
    } else if (version == MS5837_02BA21) {
        _model = MS5837_02BA;
    } else if (version == MS5837_30BA26) {
        _model = MS5837_30BA;
    } else {
        _model = MS5837_UNRECOGNISED;
    }
    // The sensor has passed the CRC check, so we should return true even if
    // the sensor version is unrecognised.
    // (The MS5637 has the same address as the MS5837 and will also pass the CRC check)
    // (but will hopefully be unrecognised.)

    add_repeating_timer_ms(REPORT_BAROMETER_PERIOD, reporter, this, &m_reporter_timer);
    return true;
}

/** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA (default)
 * and MS5837::MS5837_02BA.
 */
void MS5837::setModel(uint8_t model) {
    _model = model;
}

uint8_t MS5837::getModel() {
    return (_model);
}

/** Provide the density of the working fluid in kg/m^3. Default is for
 * seawater. Should be 997 for freshwater.
 */
void MS5837::setFluidDensity(float density) {
    fluidDensity = density;
}

/** The read from I2C takes up to 40 ms, so use sparingly is possible.
 */
void MS5837::read() {
    // D1 conversion
    if(i2c_write_blocking(ALPHA_I2C_DEFAULT, MS5837_ADDR, &MS5837_CONVERT_D1_8192, 1, false) == PICO_ERROR_GENERIC) {

    }
    busy_wait_ms(20); // MAX conversion time per datasheet
    if(i2c_write_blocking(ALPHA_I2C_DEFAULT, MS5837_ADDR, &MS5837_ADC_READ, 1, true) == PICO_ERROR_GENERIC) {

    }
    uint8_t dest_1[3];
    i2c_read_blocking(ALPHA_I2C_DEFAULT, MS5837_ADDR, dest_1, 3, false);
    D1_pres = 0;
    D1_pres = dest_1[0];
    D1_pres = (D1_pres << 8) | dest_1[1];
    D1_pres = (D1_pres << 8) | dest_1[2];

    // D2 conversion
    if(i2c_write_blocking(ALPHA_I2C_DEFAULT, MS5837_ADDR, &MS5837_CONVERT_D2_8192, 1, false) == PICO_ERROR_GENERIC) {

    }
    busy_wait_ms(20); // MAX conversion time per datasheet
    if(i2c_write_blocking(ALPHA_I2C_DEFAULT, MS5837_ADDR, &MS5837_ADC_READ, 1, true) == PICO_ERROR_GENERIC) {

    }
    uint8_t dest_2[3];
    i2c_read_blocking(ALPHA_I2C_DEFAULT, MS5837_ADDR, dest_2, 3, false);
    D2_temp = 0;
    D2_temp = dest_2[0];
    D2_temp = (D2_temp << 8) | dest_2[1];
    D2_temp = (D2_temp << 8) | dest_2[2];

    calculate();
}

/** Pressure returned in mbar or mbar*conversion rate.
 */
float MS5837::pressure(float conversion) {
    if ( _model == MS5837_02BA ) {
        return P*conversion/100.0f;
    }
    else {
        return P*conversion/10.0f;
    }
}

/** Temperature returned in deg C.
 */
float MS5837::temperature() {
    return TEMP/100.0f;
}

/** Depth returned in meters (valid for operation in incompressible
 *  liquids only. Uses density that is set for fresh or seawater.
 */
float MS5837::depth() {
    return (pressure(MS5837::Pa)-101300)/(fluidDensity*9.80665);
}

bool MS5837::reporter(struct repeating_timer *t) {
    auto _this = (MS5837*)t->user_data;
    _this->read();

    g_pressure_data.pressure = _this->pressure(MS5837::Pa);
    g_pressure_data.temperature = _this->temperature();
    g_pressure_data.depth = _this->depth();


    NMEA* msg = new NMEA();
    msg->construct("%s,%.5f,%.5f,%.5f", NMEA_BAROMETER_REPORT,
                   g_pressure_data.pressure,
                   g_pressure_data.temperature,
                   g_pressure_data.depth
    );
    std::cout << msg->get_raw() << std::endl;
    delete msg;
    return true;
}


