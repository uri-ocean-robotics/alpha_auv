#ifndef ALPHA_MCU_SERIAL_H
#define ALPHA_MCU_SERIAL_H

// PICO
#include "hardware/uart.h"
#include "hardware/gpio.h"

// STL
#include "functional"
#include "memory"
#include "iostream"

// Project
#include "nmea/nmea.h"
#include "alpha/common/dictionary.h"

class Serial {
private:

    int m_baud;

    uart_inst_t * m_hw;

public:

    Serial(int chan, int baud);

    void initialize();

    void put_string(const std::string& s);

};


#endif
