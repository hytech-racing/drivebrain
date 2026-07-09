#include <boost/asio/io_context.hpp>
#include <exception>
#include <google/protobuf/message.h>
#include <hytech_msgs.pb.h>
#include <hytech.pb.h>
#include <FoxgloveServer.hpp>
#include <MCAPLogger.hpp>
#include <mcap/writer.hpp>
#include <optional>
#include <filesystem>
#include <drivebrain_app.hpp>


std::optional<std::string> json_file; 
std::optional<std::string> dbc_file;

int parse_arguments(int &argc, char* argv[]) {
    int opt;

    while ((opt = getopt(argc, argv, "c:d:")) != -1) {
        switch (opt) {
            case 'c': 
                if (optarg != nullptr && strlen(optarg) > 0 && std::filesystem::exists(optarg)) {
                    json_file = optarg;
                } else {
                    spdlog::error("Invalid json file provided: {}", optarg);
                    return 1;
                }
                break;
            case 'd': 
                if (optarg != nullptr && strlen(optarg) > 0 && std::filesystem::exists(optarg)) {
                    dbc_file = optarg;
                } else {
                    spdlog::error("Invalid dbc file provided: {}", optarg);
                    return 1;
                }
                break;
            case '?':
                spdlog::error("Unknown command line option: -{}", static_cast<char>(optopt));
                return 1;
            default: 
                spdlog::error("Could not parse command line arguments.");
                return 1;
        }
    }

    if (!json_file) {
        spdlog::error("Did not receive the required json file argument.");
        return 1;
    }

    if (!dbc_file) {
        spdlog::error("Did not receive the required dbc file argument.");
        return 1;
    }

    return 0;
}

int main(int argc, char* argv[]) {
    // Argument Handling
    int return_code = parse_arguments(argc, argv);
    if (return_code != 0) {
        spdlog::error("Expected Usage: ./drivebrain -c path/to/config.json -d path/to/hytech.dbc");
        return return_code;
    }

    try {
        DrivebrainApp app(json_file.value(), dbc_file.value());
        app.run();
        spdlog::info("Drivebrain app created");

    } catch (const std::exception& e) {
        spdlog::error("Error in main: {}", e.what());
        return 1;
    } 

    return 0;

}
