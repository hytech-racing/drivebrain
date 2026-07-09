#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fstream> 
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <errno.h>
#include <atomic> 
#include <fcntl.h>

#include "hytech.pb.h"
#include "hytech_msgs.pb.h"

#include <dbcppp/Network.h>
#include <FoxgloveServer.hpp>
#include <MCAPLogger.hpp>


namespace comms {

class CANComms {

    public: 

        /**
         * Initializes a new can socket communications interface with the specified device name. 
         * 
         * @param device_name the name of the can device trying to be initialized
         * @param dbc_file_path the file path for the dbc 
         */
        CANComms(const std::string &device_name, const std::string &dbc_file_path);

        /**
         * Sends a prootbuf message over CAN
         * 
         * @param message the message to be sent
         */
        void send_message(std::shared_ptr<google::protobuf::Message> message);

    private: 

        /* Atomics */
        std::atomic<uint64_t> _tx_drop_count{0};
        std::atomic<uint64_t> _tx_error_count{0};

        /**
         * Receives and unpacks a can frame. 
         */
        std::optional<std::shared_ptr<google::protobuf::Message>> _decode_can_frame(struct can_frame& frame);

        /**
         * Encodes a protobuf message to a CAN message
         */
        int _encode_can_frame(std::shared_ptr<google::protobuf::Message> proto_message, can_frame* frame);


        /**
         * Receive handler that runs until the interface is destructed
         */
        void _receive_handler(); 

        /**
         * Initializes the can interface, invoked by constructor. 
         * 
         * @return 0 if successful, negative error code on failure
         */
        int _init(const std::string &dbc_file_path); 

        /**
         * Casts a can_frame struct to a protobuf message if it exists in the descriptor pool
         * 
         * @param frame the can frame to be processed
         * @return shared ptr to the protobuf message to return
         */
        std::shared_ptr<google::protobuf::Message> _protobuf_message_receive(struct can_frame &frame); 


    public: 

    private: 
        int _socket; 
        can_frame _frame;
        std::thread _reader_thread; 
        std::unordered_map<uint64_t, const dbcppp::IMessage*> _messages;
        std::unordered_map<std::string, uint64_t> _names_to_can_id;
        std::unique_ptr<dbcppp::INetwork> _net;
        std::string _device_name;
};

}
