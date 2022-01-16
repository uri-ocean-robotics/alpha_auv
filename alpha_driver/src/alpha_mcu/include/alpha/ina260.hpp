#ifndef ALPHA_INA260_H
#define ALPHA_INA260_H

#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "alpha/dictionary.h"
#include "alpha/nmea.hpp"
#include "pico/time.h"

// Declerations
class INA260 {

private:
    int m_i2c_addr;

    struct repeating_timer m_reporter_timer;

    int m_alert_pin;

public:

    INA260(int addr_ = 0x40, int alert_pin = MULTIMETER_ALERT_PIN) {
        m_i2c_addr = addr_;
        m_alert_pin = alert_pin;

    }

    void initialize() {
        // gpio_set_irq_enabled_with_callback(m_alert_pin, GPIO_IRQ_EDGE_RISE, true, &alert_callback);
        add_repeating_timer_ms(REPORT_MULTIMETER_PERIOD, reporter, this, &m_reporter_timer);
    }

    bool is_exist() {
        uint8_t p_addr = 0x00;   //Select Configuration Register.

        if(i2c_write_blocking(ALPHA_I2C_DEFAULT, m_i2c_addr, &p_addr, 1, false) == PICO_ERROR_GENERIC) {
            return false;
        }

        return true;
    }

    int rawRead(uint8_t pointer_addr,unsigned short *val_) {
        uint8_t val[2];
        if(i2c_write_blocking(ALPHA_I2C_DEFAULT, m_i2c_addr, &pointer_addr, 1, true) != PICO_ERROR_GENERIC){
            if(i2c_read_blocking(ALPHA_I2C_DEFAULT, m_i2c_addr, val, 2, false) != PICO_ERROR_GENERIC){
                *val_ = static_cast<unsigned short>(val[0]);
                *val_ = (*val_ << 8) | static_cast<unsigned short>(val[1]);
                return PICO_ERROR_NONE;
            }
        }
        return PICO_ERROR_GENERIC;
    }

    int rawWrite(char pointer_addr,unsigned short val_) {
        uint8_t val[3];
        val[0] = pointer_addr;
        val[1] = static_cast<char>((val_ >> 8) & 0x00ff);
        val[2] = static_cast<char>(val_ & 0x00ff);
        if(i2c_write_blocking(ALPHA_I2C_DEFAULT, m_i2c_addr,val,3,false) == 0){
            return 0;
        }
        return 1;
    }


    int getVoltage(double *V_){
        unsigned short val;
        if(rawRead(0x02,&val) == 0){
            *V_ = static_cast<double>(val) * 0.00125;
            return PICO_ERROR_NONE;
        }
        return PICO_ERROR_GENERIC;
    }

    int getCurrent(double *I_){
        unsigned short val;
        if(rawRead(0x01,&val) == 0){ //current register 0X1
            char *s_p = reinterpret_cast<char *>(&val);
            short d_s;
            char *d_p = reinterpret_cast<char *>(&d_s);
            *(d_p + 0) = *(s_p + 0);
            *(d_p + 1) = *(s_p + 1);
            *I_ = static_cast<double>(d_s)  * 1.25;
            return PICO_ERROR_NONE;
        }
        return PICO_ERROR_GENERIC;
    }

    int getPower(double *P_){
        unsigned short val;
        if(rawRead(0x03,&val) == 0){ //power register 0x3
            char *s_p = reinterpret_cast<char *>(&val);
            short d_s;
            char *d_p = reinterpret_cast<char *>(&d_s);
            *(d_p + 0) = *(s_p + 0);
            *(d_p + 1) = *(s_p + 1);
            *P_ = static_cast<double>(d_s)  * 10;
            return PICO_ERROR_NONE;
        }
        return PICO_ERROR_GENERIC;
    }

    int setConfig(unsigned short val){
        return rawWrite(0x00,val);  //config register 0x00
    }

    int setAlert(unsigned short val){
        return rawWrite(0x00,val);  //config register 0x00
    }

    void readAlert(void){
        unsigned short val;
        rawRead(0x06,&val);
    }

    int setLim(unsigned short val){
        return rawWrite(0x00,val);  //config register 0x00
    }

    static bool reporter(struct repeating_timer *t) {
        auto _this = (INA260*)t->user_data;

        double current, voltage, power;
        if(_this->getCurrent(&current) != PICO_ERROR_NONE) {
            current = -1;
        }
        if(_this->getVoltage(&voltage) != PICO_ERROR_NONE) {
            voltage = -1;
        }
        if(_this->getPower(&power) != PICO_ERROR_NONE) {
            power = -1;
        }

        g_multimeter_data.voltage = voltage;
        g_multimeter_data.current = current;
        g_multimeter_data.power = power;

        NMEA* msg = new NMEA();
        msg->construct("%s,%.5f,%.2f,%.1f", NMEA_MULTIMETER_REPORT, voltage, current, power);
        std::cout << msg->get_raw() << std::endl;
        delete msg;

        return true;
    }

    static void alert_callback(uint , uint32_t) {
        NMEA* msg = new NMEA();
        msg->construct("%s", NMEA_MULTIMETER_ALRT_REPORT);
        std::cout << msg->get_raw() << std::endl;
        delete msg;
    }

};

extern INA260 g_multimeter;

#endif