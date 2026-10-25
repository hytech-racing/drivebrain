#include "BlackflyComms.hpp"
#include <ImageCompression.hpp>

#include <foxglove/websocket/common.hpp>
#include <algorithm>
#include <memory>

#include "StateTracker.hpp"

using namespace comms;

BlackflyComms::BlackflyComms() {
    _running = false;
}

BlackflyComms::~BlackflyComms() {
    _running = false;
    if (_blackfly_receive_thread.joinable()) {
        _blackfly_receive_thread.join();
    }
    if (_camera)arv_camera_stop_acquisition(_camera, nullptr);
    g_object_unref(_stream);
    g_object_unref(_camera);
}

bool BlackflyComms::start(const std::string& id, const std::string& name, const std::string& pixel_format, double fps) {
    _name = name;

    // Start the camera
    _camera = arv_camera_new(id.c_str(), &_error);
    if (!_camera || !arv_camera_is_gv_device(_camera)) {
        spdlog::error("Camera is not a GigE Vision device");
        return false;
    }
    arv_camera_clear_triggers(_camera, &_error);
    arv_camera_set_pixel_format_from_string(_camera, pixel_format.c_str(), &_error);
    arv_camera_set_acquisition_mode(_camera, ARV_ACQUISITION_MODE_CONTINUOUS, &_error);
    arv_camera_set_frame_rate(_camera, fps, &_error);
    arv_camera_gv_set_packet_size(_camera, 1500, &_error);
    arv_camera_gv_set_packet_delay(_camera, 2000, &_error);
    // sync the camera clock to the jetson's PTP master (newer firmware calls it PtpEnable)
    const char* ptp_feature = arv_camera_is_feature_available(_camera, "PtpEnable", nullptr) ? "PtpEnable" : "GevIEEE1588";
    arv_camera_set_boolean(_camera, ptp_feature, TRUE, &_error);
    const auto payload = arv_camera_get_payload(_camera, &_error);
    if (!payload) {
        spdlog::error("Camera payload is empty");
        return false;
    }
    _stream = arv_camera_create_stream(_camera, nullptr, nullptr, &_error);
    if (!_stream) {
        spdlog::error("Failed to create Aravis stream");
        return false;
    }
    for (int i = 0; i < 8; ++i) {
        auto* buffer = arv_buffer_new_allocate(payload);
        if (!buffer) {
            spdlog::error("Failed to allocate camera buffer");
            return false;
        }
        arv_stream_push_buffer(_stream, buffer);
    }
    arv_camera_start_acquisition(_camera, &_error);


    if (!_camera || !_stream) {
        _running = false;
        return false;
    }

    // Start the receive thread
    _running = true;
    _blackfly_receive_thread = std::thread(&BlackflyComms::_aravis_receive_loop, this);

    return true;
}

void BlackflyComms::_aravis_receive_loop() {
    static int frame_count = 0;
    while (_running) {
        ArvBuffer *buffer = nullptr; 

        buffer = arv_stream_timeout_pop_buffer(_stream, 1000000);

        if (buffer != nullptr) {
            if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
                size_t buffer_size; 

                const void* data = arv_buffer_get_data(buffer, &buffer_size);

                int width, height;
                arv_buffer_get_image_region(buffer, nullptr, nullptr, &width, &height);

                if (width <= 0 || height <= 0 || buffer_size < static_cast<size_t>(width * height)) {
                    spdlog::error("[{}] bad frame {}x{} size {}", _name, width, height, buffer_size);
                    arv_stream_push_buffer(_stream, buffer);
                    continue;
                }

                const uint64_t timestamp_ns = arv_buffer_get_timestamp(buffer);
                const auto secs = timestamp_ns / 1000000000ULL;
                const auto nanos = timestamp_ns % 1000000000ULL;

                // Log full compressed image to mcap
                std::size_t compressed_size = 0;
                std::unique_ptr<uint8_t[]> compressed_buffer = compressBayerImage(static_cast<const uint8_t*>(data), width, height, compressed_size);

                auto mcap_image = std::make_shared<foxglove::RawImage>();
                mcap_image->mutable_timestamp()->set_seconds(secs);
                mcap_image->mutable_timestamp()->set_nanos(nanos);
                mcap_image->set_frame_id(_name);
                mcap_image->set_width(width);
                mcap_image->set_height(height);
                if (compressed_buffer) {
                    mcap_image->set_data(compressed_buffer.get(), compressed_size);
                    core::log_mcap_only(mcap_image, _name + "/raw");
                } else {
                    spdlog::error("[{}] Failed to compress camera frame", _name);
                }

                // Full resolution frame for the cone classifier, rotated 180 like the stream below (RGGB becomes BGGR)
                auto frame = std::make_shared<foxglove::RawImage>();
                *frame->mutable_timestamp() = mcap_image->timestamp();
                frame->set_frame_id(_name);
                frame->set_width(width);
                frame->set_height(height);
                frame->set_step(width);
                frame->set_encoding("bayer_bggr8");
                frame->set_data(data, width * height);
                std::reverse(frame->mutable_data()->begin(), frame->mutable_data()->end());
                core::StateTracker::instance().set_camera_frame(frame);

                // stream compressed image over foxglove stream
                cv::Mat bgr;
                // no need to resize as 2x2 averaging already halves the dimensions
                averageBayer2x2ToBgr(static_cast<const uint8_t*>(data), width, height, bgr);
                // upright like the classifier frame, so cone_boxes line up
                cv::rotate(bgr, bgr, cv::ROTATE_180);

                std::vector<uchar> jpeg_buf;
                cv::imencode(".jpg", bgr, jpeg_buf, {cv::IMWRITE_JPEG_QUALITY, 80});

                std::shared_ptr<foxglove::CompressedImage> raw_image = std::make_shared<foxglove::CompressedImage>();
                raw_image->mutable_timestamp()->set_seconds(secs);
                raw_image->mutable_timestamp()->set_nanos(nanos);
                raw_image->set_frame_id(_name);
                raw_image->set_format("jpeg");
                raw_image->set_data(jpeg_buf.data(), jpeg_buf.size());
                core::log_foxglove_only(raw_image, _name + "/compressed");
            } else {
                spdlog::error("[{}] Failed to retrieve buffer from Aravis stream, status {}", _name, static_cast<int>(arv_buffer_get_status(buffer)));
            }
            arv_stream_push_buffer(_stream, buffer);
        } else {
            spdlog::error("Failed to pop buffer from Aravis stream");
        }
        
    }
}
