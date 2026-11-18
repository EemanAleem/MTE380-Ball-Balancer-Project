#pragma once

#include <string>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

class SerialPort {
public:
    // Closes the port automatically when the object is destroyed.
    ~SerialPort();

    // Opens a COM port
    // Returns false if the port doesn't exist or another program is using it.
    bool open(const std::string& name, int baud);

    // True if the port is currently open.
    bool isOpen() const;

    // Sends text to the Arduino.
    void write(const std::string& s);

    // Reads everything the Arduino has sent back. Prints it if print is true, otherwise discards it.
    void drain(bool print);

    // Closes the port
    void close();

private:
    HANDLE h_ = INVALID_HANDLE_VALUE;   // Windows handle for the open port
};