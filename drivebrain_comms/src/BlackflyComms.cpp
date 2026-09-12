#include "BlackflyComms.hpp"

using namespace comms;

BlackflyComms::BlackflyComms() {
    _running = false;
}

BlackflyComms::~BlackflyComms() {
    arv_camera_stop_acquisition(_camera, nullptr);
    g_object_unref(_stream);
    g_object_unref(_camera);
    _running = false;
}

bool BlackflyComms::start(const std::string& id, const std::string& pixel_format, double fps) {
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
    arv_camera_gv_set_packet_size(_camera, 9000, &_error);
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

                cv::Mat bayer(cv::Size(width, height), CV_8UC1, (void*)data);
                cv::Mat bgr;
                cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2BGR);

                cv::Mat resized;
                cv::resize(bgr, resized, cv::Size(640, 640), 0, 0, cv::INTER_AREA);

                std::vector<uchar> jpeg_buf;
                cv::imencode(".jpg", resized, jpeg_buf, {cv::IMWRITE_JPEG_QUALITY, 80});
                
                std::shared_ptr<foxglove::CompressedImage> raw_image = std::make_shared<foxglove::CompressedImage>();
                auto* ts = raw_image->mutable_timestamp();
                auto now = std::chrono::system_clock::now().time_since_epoch();
                ts->set_seconds(std::chrono::duration_cast<std::chrono::seconds>(now).count());
                ts->set_nanos(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000);
                raw_image->set_frame_id("blackfly");
                raw_image->set_format("jpeg");
                raw_image->set_data(jpeg_buf.data(), jpeg_buf.size());
                core::log(raw_image);
            } else {
                spdlog::error("Failed to retrieve buffer from Aravis stream");
            }
            arv_stream_push_buffer(_stream, buffer);
        } else {
            spdlog::error("Failed to pop buffer from Aravis stream");
        }
        
    }
}
