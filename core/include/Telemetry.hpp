#pragma once

#include "StateTracker.hpp"
#include <MCAPLogger.hpp>
#include <FoxgloveServer.hpp>

namespace core {

/**
 * Global method for simultaneously logging to an MCAP and streaming to the Foxglove client.
 *
 * @msg The protobuf message to log and stream.
*/
inline void log(std::shared_ptr<google::protobuf::Message> msg) {
    MCAPLogger::instance().log_msg(msg);
    FoxgloveServer::instance().send_live_telem_msg(msg);
}

}
