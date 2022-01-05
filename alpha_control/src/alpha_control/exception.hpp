#ifndef ALPHA_CONTROL_EXCEPTION_HPP
#define ALPHA_CONTROL_EXCEPTION_HPP

#include "stdexcept"

/** @brief Exception class for Alpha ROS class
 *
 */
class control_ros_exception : public std::runtime_error {
public:
    explicit control_ros_exception(const std::string& message) : std::runtime_error(message){}
};

/** @brief Exception class for Alpha Control
 *
 */
class control_exception : public std::runtime_error {
public:
    explicit control_exception(const std::string& message) : std::runtime_error(message){}
};

#endif //ALPHA_CONTROL_EXCEPTION_HPP
