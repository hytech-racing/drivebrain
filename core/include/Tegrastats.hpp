#pragma once

#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include "hytech_msgs.pb.h"

namespace core {

std::shared_ptr<hytech_msgs::TegraStats> parse_tegrastats(const std::string& line);

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
