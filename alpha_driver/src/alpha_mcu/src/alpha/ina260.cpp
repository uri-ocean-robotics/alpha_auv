#include "alpha_common/types.h"
#include "alpha_common/dictionary.h"
#include "iostream"
#include "alpha/ina260.h"
#include "alpha/common.h"

INA260 g_multimeter = INA260();
multimeter_t g_multimeter_data;


INA260::INA260(int addr_, int alert_pin) {
    m_i2c_addr = addr_;
    m_alert_pin = alert_pin;

}

void INA260::initialize() {
    // gpio_set_irq_enabled_with_callback(m_alert_pin, GPIO_IRQ_EDGE_RISE, true, &alert_callback);
    add_repeating_timer_ms(REPORT_MULTIMETER_PERIOD, reporter, this, &m_reporter_timer);
}

bool INA260::is_exist() const {
    uint8_t p_addr = 0x00;   //Select Configuration Register.

    if(i2c_write_blocking(ALPHA_I2C_DEFAULT, m_i2c_addr, &p_addr, 1, false) == PICO_ERROR_GENERIC) {
        return false;
    }

    return true;
}

int INA260::rawRead(uint8_t pointer_addr,unsigned short *val_) const {
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

int INA260::rawWrite(char pointer_addr,unsigned short val_) const {
    uint8_t val[3];
    val[0] = pointer_addr;
    val[1] = static_cast<char>((val_ >> 8) & 0x00ff);
    val[2] = static_cast<char>(val_ & 0x00ff);
    if(i2c_write_blocking(ALPHA_I2C_DEFAULT, m_i2c_addr,val,3,false) == 0){
        return 0;
    }
    return 1;
}


int INA260::getVoltage(double *V_) const{
    unsigned short val;
    if(rawRead(0x02,&val) == 0){
        *V_ = static_cast<double>(val) * 0.00125;
        return PICO_ERROR_NONE;
    }
    return PICO_ERROR_GENERIC;
}

int INA260::getCurrent(double *I_) const{
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

int INA260::getPower(double *P_) const{
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

int INA260::setConfig(unsigned short val) const{
    return rawWrite(0x00,val);  //config register 0x00
}

int INA260::setAlert(unsigned short val) const{
    return rawWrite(0x00,val);  //config register 0x00
}

void INA260::readAlert() const{
    unsigned short val;
    rawRead(0x06,&val);
}

int INA260::setLim(unsigned short val) const{
    return rawWrite(0x00,val);  //config register 0x00
}

 bool INA260::reporter(struct repeating_timer *t) {
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

void INA260::alert_callback(uint , uint32_t) {
    NMEA* msg = new NMEA();
    msg->construct("%s", NMEA_MULTIMETER_ALRT_REPORT);
    std::cout << msg->get_raw() << std::endl;
    delete msg;
}
