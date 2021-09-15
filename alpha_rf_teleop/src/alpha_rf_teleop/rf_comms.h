#ifndef ALPHA_RF_TELEOP_RF_COMMS_H
#define ALPHA_RF_TELEOP_RF_COMMS_H

#include "boost/asio/serial_port.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"

class RfComms {
private:
    boost::asio::io_service m_io;
    boost::asio::serial_port m_serial_port;
    boost::thread m_serial_read_th;
    std::string m_port;
    int m_baud;
    void f_serial_read_loop();
    std::string f_serial_read_line();
    boost::function <void(std::string)> m_serial_callback;
    bool m_active;

public:
    RfComms();
    RfComms(std::string port, int baud);

    bool activate();
    bool deactivate();

    bool sendLine(std::string msg);

    void setCallback(boost::function<void(std::string)> func);
};


#endif //ALPHA_RF_TELEOP_RF_COMMS_H
