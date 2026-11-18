#include "serial_port.hpp"

#include <cstdio>

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& name, int baud) {
    std::string path = "\\\\.\\" + name;
    h_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h_ == INVALID_HANDLE_VALUE) return false;

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h_, &dcb)) { close(); return false; }
    dcb.BaudRate = (DWORD)baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    if (!SetCommState(h_, &dcb)) { close(); return false; }

    COMMTIMEOUTS t{};
    t.ReadIntervalTimeout = MAXDWORD;
    t.WriteTotalTimeoutConstant = 100;
    SetCommTimeouts(h_, &t);
    return true;
}

bool SerialPort::isOpen() const { return h_ != INVALID_HANDLE_VALUE; }

void SerialPort::write(const std::string& s) {
    if (!isOpen()) return;
    DWORD n = 0;
    WriteFile(h_, s.data(), (DWORD)s.size(), &n, nullptr);
}

void SerialPort::drain(bool print) {
    if (!isOpen()) return;
    char buf[256];
    DWORD n = 0;
    while (ReadFile(h_, buf, sizeof(buf), &n, nullptr) && n > 0)
        if (print) std::fwrite(buf, 1, n, stdout);
}

void SerialPort::close() {
    if (isOpen()) CloseHandle(h_);
    h_ = INVALID_HANDLE_VALUE;
}