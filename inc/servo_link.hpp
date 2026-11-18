#pragma once

#include <array>
#include <string>

#include "ik.hpp"
#include "serial_port.hpp"

class ServoLink {
public:

    // Connect to the arduino
    bool open(const std::string& port, int baud, const std::array<int, NUM_SERVOS>& channels);
    
    // Check connection to arduino
    bool isOpen() const { return port_.isOpen(); }

    // Send angles to the motors
    void send(const std::array<float, NUM_SERVOS>& anglesDeg);

    // Arduino Messages
    void poll(bool echo = false);

private:
    SerialPort port_; // Serial Port
    std::array<int, NUM_SERVOS> channels_{}; // Motors
};