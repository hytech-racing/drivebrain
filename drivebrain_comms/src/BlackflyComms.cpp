#include "BlackflyComms.hpp"
#include <ImageCompression.hpp>
#include "CameraSync.hpp"

#include <chrono>
#include <foxglove/websocket/common.hpp>
#include <memory>
#include <cmath>

#if JETSON_ENABLED
#include <foxglove/ImageAnnotations.pb.h>
#endif

using namespace comms;

bool BlackflyComms::_check_error(const char* operation) {
    if (!_error) return true;
    spdlog::error("[{}] {}: {}", _name, operation, _error->message);
    g_clear_error(&_error);
    return false;
}

bool BlackflyComms::_ptp_locked(int64_t& parent) {
    arv_camera_execute_command(_camera, "GevIEEE1588DataSetLatch", &_error);
    if (!_check_error("latch PTP dataset")) return false;
    const char* status = arv_camera_get_string(_camera, "GevIEEE1588StatusLatched", &_error);
    if (!_check_error("read PTP status") || !status || std::string(status) != "Slave") return false;
    const auto offset = arv_camera_get_integer(_camera, "GevIEEE1588OffsetFromMasterLatched", &_error);
    parent = arv_camera_get_integer(_camera, "GevIEEE1588ParentClockIdLatched", &_error);
    return _check_error("read PTP offset/parent") && parent != 0 &&
           offset >= -camera_sync::max_offset_ns && offset <= camera_sync::max_offset_ns;
}

uint64_t BlackflyComms::_camera_time() {
    // Standard GigE Vision timestamp latch registers, independent of vendor
    // GenICam names. Timestamps must already be confirmed to tick at 1 GHz.
    auto* device = arv_camera_get_device(_camera);
    arv_device_write_register(device, 0x944, 2, &_error);
    if (!_check_error("latch camera timestamp")) return 0;
    uint32_t high = 0, low = 0;
    arv_device_read_register(device, 0x948, &high, &_error);
    arv_device_read_register(device, 0x94c, &low, &_error);
    if (!_check_error("read camera timestamp")) return 0;
    return (uint64_t(high) << 32) | low;
}

bool BlackflyComms::_schedule_frame(uint64_t timestamp) {
    if (++_action_request == 0) ++_action_request;
    const auto packet = camera_sync::scheduled_action(_action_request, timestamp);
    const auto sent = g_socket_send(_action_socket, reinterpret_cast<const char*>(packet.data()),
                                    packet.size(), nullptr, &_error);
    if (!_check_error("send scheduled action") || sent != packet.size()) return false;
    // No retry: executing a duplicate action can create an extra exposure.
    const auto deadline = g_get_monotonic_time() + 20000;
    while (g_get_monotonic_time() < deadline) {
        const auto remaining = deadline - g_get_monotonic_time();
        if (remaining <= 0) return false;
        if (!g_socket_condition_timed_wait(_action_socket, G_IO_IN,
                                           remaining, nullptr, &_error)) {
            _check_error("wait for scheduled action acknowledgement");
            return false;
        }
        uint8_t ack[64]{};
        const auto size = g_socket_receive(_action_socket, reinterpret_cast<char*>(ack),
                                           sizeof(ack), nullptr, &_error);
        if (!_check_error("receive action acknowledgement")) return false;
        if (size < 8 || ack[6] != uint8_t(_action_request >> 8) ||
            ack[7] != uint8_t(_action_request)) continue;
        if (camera_sync::action_accepted(ack, size, _action_request)) return true;
        spdlog::error("[{}] Scheduled action rejected, status 0x{:02x}{:02x}", _name, ack[0], ack[1]);
        return false;
    }
    return false;
}

void BlackflyComms::stop_synchronization() {
    _sync_running = false;
    if (_sync_thread.joinable()) _sync_thread.join();
}

bool BlackflyComms::synchronize(const std::vector<std::unique_ptr<BlackflyComms>>& cameras) {
    if (cameras.empty()) return false;
    std::vector<BlackflyComms*> group;
    for (const auto& camera : cameras) {
        if (!camera || !camera->_running || camera->_fps != cameras.front()->_fps ||
            camera->_sync_thread.joinable()) return false;
        group.push_back(camera.get());
    }
    // Require three consecutive samples with a common parent and <=100 us
    // offset per camera. Merely enabling PTP does not establish lock.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    int stable = 0;
    while (stable < 3 && std::chrono::steady_clock::now() < deadline) {
        bool locked = true;
        int64_t parent = 0;
        for (auto* camera : group) {
            int64_t current_parent = 0;
            if (!camera->_ptp_locked(current_parent) || (parent && parent != current_parent)) locked = false;
            parent = current_parent;
        }
        stable = locked ? stable + 1 : 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (stable < 3) {
        spdlog::error("Camera synchronization failed: no shared PTP master/lock within 30 seconds");
        return false;
    }
    auto* owner = cameras.front().get();
    owner->_sync_running = true;
    owner->_sync_thread = std::thread(&BlackflyComms::_synchronize_loop, owner, std::move(group));
    spdlog::info("Armed {} cameras for PTP scheduled capture at {} Hz", cameras.size(), owner->_fps);
    return true;
}

void BlackflyComms::_synchronize_loop(std::vector<BlackflyComms*> cameras) {
    using namespace std::chrono;
    const uint64_t period = std::llround(camera_sync::ns_per_second / _fps);
    uint64_t next = 0;
    auto check_at = steady_clock::now();
    while (_sync_running) {
        if (steady_clock::now() >= check_at) {
            int64_t parent = 0;
            for (auto* camera : cameras) {
                int64_t current_parent = 0;
                if (!camera->_ptp_locked(current_parent) || (parent && parent != current_parent)) {
                    spdlog::error("Camera PTP lock lost; stopping scheduled capture");
                    _sync_running = false;
                    break;
                }
                parent = current_parent;
            }
            check_at = steady_clock::now() + seconds(1);
        }
        if (!_sync_running) break;
        const auto now = _camera_time();
        if (!now) break;
        if (next > now + camera_sync::lead_ns + 2 * period) {
            spdlog::error("Camera clock moved backwards; stopping scheduled capture");
            break;
        }
        if (!next) next = now + camera_sync::lead_ns;
        // Queue ahead of execution so packet arrival order does not determine
        // exposure time. After a host stall, skip slots for the entire group.
        if (next < now + camera_sync::lead_ns / 2) {
            const auto skipped = (now + camera_sync::lead_ns - next) / period + 1;
            next += skipped * period;
            spdlog::warn("Camera scheduler overrun: skipped {} shared trigger slots", skipped);
        }
        if (next <= now + camera_sync::lead_ns) {
            for (auto* camera : cameras) {
                if (!camera->_schedule_frame(next)) {
                    spdlog::error("Camera trigger failed; stopping group capture");
                    _sync_running = false;
                    break;
                }
            }
            next += period;
        }
        std::this_thread::sleep_for(milliseconds(5));
    }
    _sync_running = false;
    // Cancel any actions already queued when stopping or losing PTP lock.
    for (auto* camera : cameras) {
        arv_camera_execute_command(camera->_camera, "ActionQueueEmpty", &camera->_error);
        camera->_check_error("clear scheduled actions");
    }
}

BlackflyComms::BlackflyComms() {
    _running = false;
}

BlackflyComms::~BlackflyComms() {
    stop_synchronization();
    _running = false;
    if (_blackfly_receive_thread.joinable()) {
        _blackfly_receive_thread.join();
    }
    if (_camera)arv_camera_stop_acquisition(_camera, nullptr);
    if (_action_socket) g_object_unref(_action_socket);
    if (_stream) g_object_unref(_stream);
    if (_camera) g_object_unref(_camera);
    g_clear_error(&_error);
}

bool BlackflyComms::start(const std::string& id, const std::string& name, const std::string& pixel_format, double fps) {
    _name = name;
    if (!std::isfinite(fps) || fps <= 0 || fps > 30) {
        spdlog::error("[{}] Synchronization supports frame rates in (0, 30] Hz", _name);
        return false;
    }
    _fps = fps;

    // Start the camera
    _camera = arv_camera_new(id.c_str(), &_error);
    if (!_camera || !arv_camera_is_gv_device(_camera)) {
        spdlog::error("Camera is not a GigE Vision device");
        return false;
    }
    if (!_check_error("open camera")) return false;
    arv_camera_clear_triggers(_camera, &_error);
    arv_camera_set_pixel_format_from_string(_camera, pixel_format.c_str(), &_error);
    arv_camera_set_acquisition_mode(_camera, ARV_ACQUISITION_MODE_CONTINUOUS, &_error);
    arv_camera_set_frame_rate(_camera, fps, &_error);
    arv_camera_gv_set_packet_size(_camera, 1500, &_error);
    arv_camera_gv_set_packet_delay(_camera, 2000, &_error);
    if (!_check_error("configure image acquisition")) return false;
    // Require an external PTP master; a camera elected master at boot may have
    // an arbitrary epoch. Never silently fall back to unsynchronized acquisition.
    const char* ptp_feature = arv_camera_is_feature_available(_camera, "PtpEnable", nullptr) ? "PtpEnable" : "GevIEEE1588";
    arv_camera_set_boolean(_camera, ptp_feature, TRUE, &_error);
    arv_camera_set_string(_camera, "GevIEEE1588Mode", "SlaveOnly", &_error);
    if (!_check_error("enable PTP slave mode")) return false;
    arv_camera_set_string(_camera, "TriggerSelector", "FrameStart", &_error);
    arv_camera_set_string(_camera, "TriggerMode", "Off", &_error);
    arv_camera_set_string(_camera, "TriggerSource", "Action0", &_error);
    arv_camera_set_float(_camera, "TriggerDelay", 0.0, &_error);
    arv_camera_set_boolean(_camera, "AcquisitionFrameRateEnable", FALSE, &_error);
    arv_camera_set_integer(_camera, "ActionDeviceKey", camera_sync::device_key, &_error);
    arv_camera_set_integer(_camera, "ActionGroupKey", camera_sync::group_key, &_error);
    arv_camera_set_integer(_camera, "ActionGroupMask", camera_sync::group_mask, &_error);
    arv_camera_execute_command(_camera, "ActionQueueEmpty", &_error);
    arv_camera_set_string(_camera, "TriggerMode", "On", &_error);
    if (!_check_error("arm scheduled Action0 frame triggers")) return false;
    auto* device = ARV_GV_DEVICE(arv_camera_get_device(_camera));
    const auto frequency = arv_gv_device_get_timestamp_tick_frequency(device, &_error);
    if (!_check_error("read timestamp frequency") || frequency != camera_sync::ns_per_second) {
        spdlog::error("[{}] Scheduled capture requires a 1 GHz timestamp clock", _name);
        return false;
    }
    const auto queue_size = arv_camera_get_integer(_camera, "ActionQueueSize", &_error);
    if (!_check_error("read scheduled action queue size")) return false;
    if (queue_size < 8) {
        spdlog::error("[{}] Scheduled capture needs at least 8 action queue entries", _name);
        return false;
    }
    _action_socket = g_socket_new(G_SOCKET_FAMILY_IPV4, G_SOCKET_TYPE_DATAGRAM,
                                  G_SOCKET_PROTOCOL_UDP, &_error);
    if (!_check_error("create action socket")) return false;
    auto* interface = arv_gv_device_get_interface_address(device);
    auto* source = g_inet_socket_address_new(
        g_inet_socket_address_get_address(G_INET_SOCKET_ADDRESS(interface)), 0);
    g_socket_bind(_action_socket, source, FALSE, &_error);
    g_object_unref(source);
    if (!_check_error("bind action socket")) return false;
    auto* address = arv_gv_device_get_device_address(device);
    auto* destination = g_inet_socket_address_new(
        g_inet_socket_address_get_address(G_INET_SOCKET_ADDRESS(address)), 3956);
    g_socket_connect(_action_socket, destination, nullptr, &_error);
    g_object_unref(destination);
    if (!_check_error("connect action socket")) return false;
    g_socket_set_blocking(_action_socket, FALSE);
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
    if (!_check_error("start acquisition")) return false;


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

                // Use the camera's capture timestamp, independent of Ethernet,
                // JPEG encoding, detection, and receive-thread scheduling delays.
                const auto timestamp = arv_buffer_get_timestamp(buffer);
                if (timestamp == 0) {
                    spdlog::error("[{}] Dropping frame with missing camera timestamp", _name);
                    arv_stream_push_buffer(_stream, buffer);
                    continue;
                }
                const auto secs = timestamp / camera_sync::ns_per_second;
                const auto nanos = timestamp % camera_sync::ns_per_second;

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
                    // core::log_mcap_only(mcap_image, _name + "/raw");
                } else {
                    spdlog::error("[{}] Failed to compress camera frame", _name);
                }


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

                auto perception_image = std::make_shared<foxglove::RawImage>();
                perception_image->mutable_timestamp()->set_seconds(secs);
                perception_image->mutable_timestamp()->set_nanos(nanos);
                perception_image->set_frame_id(_name);
                perception_image->set_width(bgr.cols);
                perception_image->set_height(bgr.rows);
                perception_image->set_encoding("bgr8");
                perception_image->set_step(static_cast<uint32_t>(bgr.step[0]));
                perception_image->set_data(bgr.data, bgr.step[0] * bgr.rows);
                core::log_mcap_only(perception_image, _name + "/perception");
                
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

                    // core::log_mcap_only(make_annotations(1.0f, true), _name + "/detections");                                     // matches <name>/raw
                    core::log_foxglove_only(make_annotations(2.0f, false), _name + "/detections"); // matches <name>/compressed
                }
#endif

            } else {
                spdlog::error("[{}] Failed to retrieve buffer from Aravis stream, status {}", _name, static_cast<int>(arv_buffer_get_status(buffer)));
            }
            arv_stream_push_buffer(_stream, buffer);
        } else {
            spdlog::error("Failed to pop buffer from Aravis stream");
        }
        
    }
}
