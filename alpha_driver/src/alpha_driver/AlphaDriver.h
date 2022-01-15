#ifndef ALPHA_DRIVER_ALPHADRIVER_H
#define ALPHA_DRIVER_ALPHADRIVER_H

#include "boost/asio/serial_port.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"

#define NMEA_PWM_CMD "PWMC2"
#define NMEA_THRUST_PWM_CMD "PWMC"
#define NMEA_THRUST_PWM_REPORT "PWMR"
#define NMEA_BAROMETER_REPORT "BARR"
#define NMEA_MULTIMETER_REPORT "MULR"

class AlphaDriver{
protected:

    boost::asio::io_service m_io;

    boost::asio::serial_port m_serial_port;

    boost::thread m_serial_read_th;

    std::string m_port;

    int m_baud;

    bool m_active;

    boost::function <void(std::string)> m_serial_callback;

    void f_serial_read_loop();

    bool f_serial_send_line(std::string msg);

    std::string f_serial_read_line();
    
public:
    AlphaDriver();

    AlphaDriver(std::string port, int baud);

    void initialize();

    void send_raw(std::string nmea);

    void set_port(decltype(m_port) val) { m_port = val;}
    auto get_port() -> decltype(m_port) {return m_port; }

    void set_baud(decltype(m_baud) val) { m_baud = val;}
    auto get_baud() -> decltype(m_baud) {return m_baud;}

    void set_serial_callback(decltype(m_serial_callback) c) { m_serial_callback  = c;}
    auto get_serial_callback() -> decltype(m_serial_callback) {return m_serial_callback;}

    void cmd_pwm(double pwm, uint8_t channel);
};


#endif //ALPHA_DRIVER_ALPHADRIVER_H
