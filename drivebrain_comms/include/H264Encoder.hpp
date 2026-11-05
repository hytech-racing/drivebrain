#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <opencv2/imgproc.hpp>
extern "C" {
#include <x264.h>
}

namespace comms {

// One Annex-B access unit per call, with no delayed frames. Each frame includes
// its headers and is independently decodable, including after MCAP rotation.
class H264Encoder {
public:
    H264Encoder(int width, int height) {
        x264_param_t params;
        x264_param_default_preset(&params, "ultrafast", "zerolatency");
        params.i_width = width;
        params.i_height = height;
        params.i_csp = X264_CSP_I420;
        params.i_keyint_max = 1;
        params.b_repeat_headers = 1;
        params.b_annexb = 1;
        params.rc.i_rc_method = X264_RC_CRF;
        params.rc.f_rf_constant = 23;
        encoder.reset(x264_encoder_open(&params));
        if (!encoder) throw std::runtime_error("Could not open H.264 encoder");
    }

    std::string encode(const cv::Mat& bgr) {
        cv::Mat yuv;
        cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV_I420);
        x264_picture_t input, output;
        x264_picture_init(&input);
        input.img.i_csp = X264_CSP_I420;
        input.img.i_plane = 3;
        const auto pixels = bgr.total();
        input.img.plane[0] = yuv.data;
        input.img.plane[1] = yuv.data + pixels;
        input.img.plane[2] = yuv.data + pixels + pixels / 4;
        input.img.i_stride[0] = bgr.cols;
        input.img.i_stride[1] = input.img.i_stride[2] = bgr.cols / 2;
        input.i_pts = sequence++;
        x264_nal_t* nals;
        int count;
        const int size = x264_encoder_encode(encoder.get(), &nals, &count, &input, &output);
        if (size <= 0) throw std::runtime_error("Could not encode H.264 frame");
        return std::string(reinterpret_cast<char*>(nals[0].p_payload), size);
    }

private:
    std::unique_ptr<x264_t, decltype(&x264_encoder_close)> encoder{nullptr, x264_encoder_close};
    int64_t sequence = 0;
};

} // namespace comms
