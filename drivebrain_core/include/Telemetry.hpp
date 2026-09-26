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

/**
 * Logs the given protobuf message to the MCAP file only.
 * 
 * @msg The protobuf message to log to the MCAP file only.
 */
inline void log_mcap_only(std::shared_ptr<google::protobuf::Message> msg) {
    MCAPLogger::instance().log_msg(msg);
}


/**
 * Logs the given protobuf message to the Foxglove client only.
 * 
 * @msg The protobuf message to log to the Foxglove client only.
 */
inline void log_foxglove_only(std::shared_ptr<google::protobuf::Message> msg) {
    FoxgloveServer::instance().send_live_telem_msg(msg);
}

}
