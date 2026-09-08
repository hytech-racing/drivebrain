#pragma once

#include <memory>

namespace comms {

class FLIRDriver {
public:
    explicit FLIRDriver(bool& init_not_successful);
    ~FLIRDriver();
    FLIRDriver(const FLIRDriver&) = delete;
    FLIRDriver& operator=(const FLIRDriver&) = delete;

    bool init();
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

}
