#ifndef ALPHA_DRIVER_ALPHADRIVER_H
#define ALPHA_DRIVER_ALPHADRIVER_H

#include "boost/asio/serial_port.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"

#define NMEA_THRUST_PWM_CMD "PWMC"
#define NMEA_THRUST_PWM_REPORT "PWMR"
#define NMEA_BAROMETER_REPORT "BARR"
#define NMEA_MULTIMETER_REPORT "MULR"

class AlphaDriver{
protected:

    boost::asio::io_service io_;
    boost::asio::serial_port serial_port_;

    boost::thread serial_read_th_;

    std::string port_;
    int baud_;

    bool active_;

    boost::function <void(std::string)> serial_callback_;

    void serialReadLoop();

    bool serialSendLine(std::string msg);

    std::string serialReadLine();
    
public:
    AlphaDriver();

    AlphaDriver(std::string port, int baud);

    void initialize();

    void sendRaw(std::string nmea);

    void setPort(decltype(port_) val) { port_ = val;}
    auto getPort() -> decltype(port_) {return port_; }

    void setBaud(decltype(baud_) val) { baud_ = val;}
    auto getBaud() -> decltype(baud_) {return baud_;}

    void setSerialCallback(decltype(serial_callback_) c) {serial_callback_  = c;}
    auto getSerialCallback() -> decltype(serial_callback_) {return serial_callback_;}

    void cmdPwm(double x, double y, double z);

};


#endif //ALPHA_DRIVER_ALPHADRIVER_H
