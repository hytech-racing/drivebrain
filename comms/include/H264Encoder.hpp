#pragma once

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <opencv2/imgproc.hpp>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

namespace comms {

// One independently decodable Annex-B access unit per call, including headers.
class H264Encoder {
public:
    H264Encoder(int width, int height, double fps) {
        static std::once_flag init;
        std::call_once(init, [] { gst_init(nullptr, nullptr); });
        int numerator, denominator;
        gst_util_double_to_fraction(fps, &numerator, &denominator);
        const std::string description =
            "appsrc name=input is-live=true format=time do-timestamp=true "
            "caps=\"video/x-raw,format=I420,width=" + std::to_string(width) +
            ",height=" + std::to_string(height) + ",framerate=" +
            std::to_string(numerator) + "/" + std::to_string(denominator) +
            "\" ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
            "! nvv4l2h264enc bitrate=8000000 iframeinterval=1 idrinterval=1 "
            "insert-sps-pps=true num-B-Frames=0 "
            "! video/x-h264,stream-format=byte-stream,alignment=au "
            "! appsink name=output sync=false async=false max-buffers=1";
        GError* error = nullptr;
        pipeline.reset(gst_parse_launch(description.c_str(), &error));
        if (error) {
            const std::string message = error->message;
            g_error_free(error);
            throw std::runtime_error(message);
        }
        if (!pipeline) throw std::runtime_error("Could not create H.264 pipeline");
        source.reset(gst_bin_get_by_name(GST_BIN(pipeline.get()), "input"));
        sink.reset(gst_bin_get_by_name(GST_BIN(pipeline.get()), "output"));
        if (!source || !sink || gst_element_set_state(pipeline.get(), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            gst_element_set_state(pipeline.get(), GST_STATE_NULL);
            throw std::runtime_error("Could not start H.264 pipeline");
        }
    }

    ~H264Encoder() {
        if (pipeline) gst_element_set_state(pipeline.get(), GST_STATE_NULL);
    }

    std::string encode(const cv::Mat& bgr) {
        cv::Mat yuv;
        cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV_I420);
        auto* buffer = gst_buffer_new_allocate(nullptr, yuv.total(), nullptr);
        if (!buffer) throw std::runtime_error("Could not allocate video buffer");
        gst_buffer_fill(buffer, 0, yuv.data, yuv.total());
        // appsrc takes ownership of buffer, including on failure.
        if (gst_app_src_push_buffer(GST_APP_SRC(source.get()), buffer) != GST_FLOW_OK)
            throw std::runtime_error("Could not submit H.264 frame");
        std::unique_ptr<GstSample, decltype(&gst_sample_unref)> sample(
            gst_app_sink_try_pull_sample(GST_APP_SINK(sink.get()), GST_SECOND), gst_sample_unref);
        if (!sample) throw std::runtime_error("H.264 encoder timed out or stopped");
        auto* encoded = gst_sample_get_buffer(sample.get());
        GstMapInfo map;
        if (!gst_buffer_map(encoded, &map, GST_MAP_READ))
            throw std::runtime_error("Could not read H.264 frame");
        std::string result(reinterpret_cast<char*>(map.data), map.size);
        gst_buffer_unmap(encoded, &map);
        return result;
    }

private:
    using Element = std::unique_ptr<GstElement, decltype(&gst_object_unref)>;
    Element pipeline{nullptr, gst_object_unref};
    Element source{nullptr, gst_object_unref};
    Element sink{nullptr, gst_object_unref};
};

} // namespace comms
