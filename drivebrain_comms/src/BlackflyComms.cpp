#include "BlackflyComms.hpp"
#include <DataCompression.hpp>

#include <chrono>
#include <foxglove/websocket/common.hpp>
#include <memory>

#if JETSON_ENABLED
#include <foxglove/ImageAnnotations.pb.h>
#endif

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

#if JETSON_ENABLED
    try {
        // FOLLOW UP: depending on performance we may want to run 
        // the detector in a seperate thread and pop off of a queue
        // but we'll see how this does.
        _detector = std::make_unique<Detector>("/home/hytech/engines/cones.engine");
    } catch (const std::exception& e) {
        spdlog::error("[{}] Cone detection disabled: {}", _name, e.what());
    }
#endif

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

                auto now = std::chrono::system_clock::now().time_since_epoch();
                auto secs = std::chrono::duration_cast<std::chrono::seconds>(now).count();
                auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000;

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


#if JETSON_ENABLED
                if (_detector) {
                    const auto detections = _detector->detect(bgr, 0.5f);
                    auto make_annotations = [&](float scale, bool unrotate) {
                        auto annotations = std::make_shared<foxglove::ImageAnnotations>();
                        for (const auto& det : detections) {
                            cv::Rect r = det.bbox;
                            if (unrotate) {
                                r.x = width - (r.x + r.width);
                                r.y = height - (r.y + r.height);
                            }
                            const float x0 = r.x * scale, y0 = r.y * scale;
                            const float x1 = (r.x + r.width) * scale, y1 = (r.y + r.height) * scale;
                            const cv::Scalar c = _detector->cone_color(det.class_id); // BGR, 0-255

                            auto* box = annotations->add_points();
                            box->mutable_timestamp()->set_seconds(secs);
                            box->mutable_timestamp()->set_nanos(nanos);
                            box->set_type(foxglove::PointsAnnotation::LINE_LOOP);
                            const cv::Point2f corners[] = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
                            for (const auto& corner : corners) {
                                auto* p = box->add_points();
                                p->set_x(corner.x);
                                p->set_y(corner.y);
                            }
                            auto* color = box->mutable_outline_color(); // RGBA, 0-1
                            color->set_r(c[2] / 255.0);
                            color->set_g(c[1] / 255.0);
                            color->set_b(c[0] / 255.0);
                            color->set_a(1.0);
                            box->set_thickness(2);

                            auto* label = annotations->add_texts();
                            label->mutable_timestamp()->set_seconds(secs);
                            label->mutable_timestamp()->set_nanos(nanos);
                            label->mutable_position()->set_x(x0);
                            label->mutable_position()->set_y(y0);
                            label->set_text(cv::format("%s %.2f", _detector->cone_name(det.class_id), det.score));
                            label->set_font_size(12);
                            *label->mutable_text_color() = *color;
                            label->mutable_background_color()->set_a(0.5); // translucent black
                        }
                        return annotations;
                    };

                    core::log_mcap_only(make_annotations(1.0f, true), _name + "/detections");                                     // matches <name>/raw
                    core::log_foxglove_only(make_annotations(static_cast<float>(out_width) / width, false), _name + "/detections"); // matches <name>/compressed
                }
#endif
                // stream compressed image over foxglove stream
                cv::Mat bgr;
                // no need to resize as 2x2 averaging already halves the dimensions
                averageBayer2x2ToBgr(static_cast<const uint8_t*>(data), width, height, bgr);
                cv::rotate(bgr, bgr, cv::ROTATE_180); // TODO apply this in camera settings

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
