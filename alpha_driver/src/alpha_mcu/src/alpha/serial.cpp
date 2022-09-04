
#include "serial.h"

Serial::Serial(int chan, int baud) {

    if(chan == 0) {
        m_hw = uart0;
    } else if (chan == 1) {
        m_hw = uart1;
    }

    m_baud = baud;

}

void uart_callback(uart_inst_t* u) {
    // read it into std::string
    std::string in;
    while (uart_is_readable(u)) {
        in.push_back(uart_getc(u));
    }

    NMEA* msg = new NMEA();

    msg->construct("%s,%s",
                   u == uart0 ? NMEA_SERIAL0 : NMEA_SERIAL1,
                   in.c_str());

    std::cout << msg->get_raw() << std::endl;
    delete msg;
}

void uart0_callback() {
    uart_callback(uart0);
}

void uart1_callback() {
    uart_callback(uart1);
}

void Serial::initialize() {

    int uart_irq;
    int rx_pin;
    int tx_pin;
    irq_handler_t irq;

    if(m_hw == uart0) {
        uart_irq = UART0_IRQ;
        rx_pin = 0;
        tx_pin = 1;
        irq = &uart0_callback;
    } else if (m_hw == uart1) {
        uart_irq = UART1_IRQ;
        rx_pin = 8;
        tx_pin = 9;
        irq = &uart1_callback;
    }

    uart_init(m_hw, m_baud);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    // irq_set_exclusive_handler(uart_irq, irq);
    // irq_set_enabled(uart_irq, true);
}

void Serial::put_string(const std::string &s) {

    uart_puts(m_hw, s.c_str());

}