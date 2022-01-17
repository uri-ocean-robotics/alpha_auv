#ifndef ALPHA_INA260_H
#define ALPHA_INA260_H

#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "alpha/dictionary.h"
#include "nmea/nmea.h"
#include "pico/time.h"

// Declerations
class INA260 {

private:
    int m_i2c_addr;

    struct repeating_timer m_reporter_timer;

    int m_alert_pin;

public:

    INA260(int addr_ = 0x40, int alert_pin = MULTIMETER_ALERT_PIN);

    void initialize();

    bool is_exist() const;

    int rawRead(uint8_t pointer_addr,unsigned short *val_) const;

    int rawWrite(char pointer_addr,unsigned short val_) const;

    int getVoltage(double *V_) const;

    int getCurrent(double *I_) const;

    int getPower(double *P_) const;

    int setConfig(unsigned short val) const;

    int setAlert(unsigned short val) const;

    void readAlert() const;

    int setLim(unsigned short val) const;

    static bool reporter(struct repeating_timer *t);

    static void alert_callback(uint , uint32_t);

};

extern INA260 g_multimeter;

extern multimeter_t g_multimeter_data;

#endif