#include "servo_link.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <thread>

bool ServoLink::open(const std::string& port, int baud, const std::array<int, NUM_SERVOS>& channels) {
    channels_ = channels;
    if (!port_.open(port, baud)) return false;
    std::printf("Arduino Connected\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    port_.drain(false);
    return true;
}

void ServoLink::send(const std::array<float, NUM_SERVOS>& anglesDeg) {
    if (!port_.isOpen()) return;
    std::string msg;
    char line[32];
    for (int i = 0; i < NUM_SERVOS; i++) {
        float a = std::clamp(anglesDeg[i], 0.0f, 180.0f);
        std::snprintf(line, sizeof(line), "%d %.1f\n", channels_[i], a);
        msg += line;
    }
    port_.write(msg);
}

void ServoLink::poll(bool echo) { port_.drain(echo); }
