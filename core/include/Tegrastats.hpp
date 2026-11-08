#pragma once

#include <cstdio>
#include <string>
#include <thread>
#include <unordered_map>

namespace core {

std::unordered_map<std::string, double> parse_tegrastats(const std::string& line);

class Tegrastats {
public:
    Tegrastats();
    ~Tegrastats();
    Tegrastats(const Tegrastats&) = delete;
    Tegrastats& operator=(const Tegrastats&) = delete;

private:
    int _pid = -1;
    FILE* _output = nullptr;
    std::thread _thread;
};

}
