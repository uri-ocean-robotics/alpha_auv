#include "AlphaDriver.h"
#include "nmea.h"
#include "iostream"


AlphaDriver::AlphaDriver() :
    io_(),
    serial_port_(io_)
{

}

AlphaDriver::AlphaDriver(std::string port, int baud) :
    io_(),
    serial_port_(io_),
    port_(port),
    baud_(baud),
    active_(true)
{

   serial_port_.open(port);
   serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud));

   serial_read_th_ = boost::thread(boost::bind(&AlphaDriver::serialReadLoop, this));
}

void AlphaDriver::initialize() {

}

std::string AlphaDriver::serialReadLine() {
    char buf[BUFSIZ];
    memset(buf, 0, BUFSIZ);
    char c;
    for(int i = 0 ; i < BUFSIZ; i++) {
        boost::asio::read(serial_port_, boost::asio::buffer(&c, 1));
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

void AlphaDriver::serialReadLoop() {
    while (active_) {
        auto line = serialReadLine();
        if(serial_callback_) {
            serial_callback_(line);
        }
    }
}

bool AlphaDriver::serialSendLine(std::string msg) {
    msg += "\r\n";
    return boost::asio::write(serial_port_, boost::asio::buffer(msg.c_str(), msg.size())) == msg.size();
}

void AlphaDriver::sendRaw(std::string nmea) {
    serialSendLine(nmea);
}

void AlphaDriver::cmdPwm(uint16_t x, uint16_t y, uint16_t z) {
    NMEA thrust_msg;
    
    thrust_msg.construct("%s,%d,%d,%d", NMEA_THRUST_PWM_CMD, x, y, z);
    
    serialSendLine(std::string(thrust_msg.get_raw()));
}
