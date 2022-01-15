#include "AlphaDriver.h"
#include "nmea.h"
#include "iostream"


AlphaDriver::AlphaDriver() :
        m_io(),
        m_serial_port(m_io)
{

}

AlphaDriver::AlphaDriver(std::string port, int baud) :
        m_io(),
        m_serial_port(m_io),
        m_port(port),
        m_baud(baud),
        m_active(true)
{

   m_serial_port.open(port);
   m_serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud));

    m_serial_read_th = boost::thread(boost::bind(&AlphaDriver::f_serial_read_loop, this));
}

void AlphaDriver::initialize() {

}

std::string AlphaDriver::f_serial_read_line() {
    char buf[BUFSIZ];
    memset(buf, 0, BUFSIZ);
    char c;
    for(int i = 0 ; i < BUFSIZ; i++) {
        boost::asio::read(m_serial_port, boost::asio::buffer(&c, 1));
        switch (c) {
            case '\r':
                break;
            case '\n':
                return std::string(buf);
            default:
                buf[i] = c;
        }
    }
    return std::string(buf);
}

void AlphaDriver::f_serial_read_loop() {
    while (m_active) {
        auto line = f_serial_read_line();
        if(m_serial_callback) {
            m_serial_callback(line);
        }
    }
}

bool AlphaDriver::f_serial_send_line(std::string msg) {
    msg += "\r\n";
    return boost::asio::write(m_serial_port, boost::asio::buffer(msg.c_str(), msg.size())) == msg.size();
}

void AlphaDriver::send_raw(std::string nmea) {
    f_serial_send_line(nmea);
}

void AlphaDriver::cmd_pwm(double pwm, uint8_t channel) {
    NMEA pwm_msg;

    pwm_msg.construct("%s,%d,%.5f", NMEA_PWM_CMD, pwm, channel);
}