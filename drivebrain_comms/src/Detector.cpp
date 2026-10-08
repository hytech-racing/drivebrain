#include "Detector.hpp"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <fstream>
#include <stdexcept> 

using namespace comms;

namespace {
struct Logger : nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            spdlog::warn("[TensorRT] {}", msg);
        }
    }
} g_logger;
}

Detector::Detector(const std::string& engine_path) {
    std::ifstream file(engine_path, std::ios::binary);
    std::vector<char> blob((std::istreambuf_iterator<char>(file)), {});

    if (!file) {
        throw std::runtime_error("Failed to open engine file: " + engine_path);
    }

    _runtime.reset(nvinfer1::createInferRuntime(g_logger));
    _engine.reset(_runtime->deserializeCudaEngine(blob.data(), blob.size()));
    if (!_engine) throw std::runtime_error("Failed to create CUDA engine from file: " + engine_path);
    _context.reset(_engine->createExecutionContext());

    const char* input_name = _engine->getIOTensorName(0);
    const char* output_name = _engine->getIOTensorName(1);
    _input_h = _engine->getTensorShape(input_name).d[2];
    _input_w = _engine->getTensorShape(input_name).d[3];
    _num_dets = _engine->getTensorShape(output_name).d[1];

    cudaStreamCreate(&_stream);
    cudaMalloc(&_input_dev, _input_h * _input_w * 3 * sizeof(float));
    cudaMalloc(&_output_dev, _num_dets * 6 * sizeof(float));

    _context->setTensorAddress(input_name, _input_dev);
    _context->setTensorAddress(output_name, _output_dev);

    spdlog::info("Loaded {} ({}x{} input)", engine_path, _input_w, _input_h);
}

Detector::~Detector() {
    cudaFree(_input_dev);
    cudaFree(_output_dev);
    cudaStreamDestroy(_stream);
}

std::vector<Detection> Detector::detect(const cv::Mat& bgr, float min_score) {
    cv::Mat img;
    cv::resize(bgr, img, cv::Size(_input_w, _input_h));
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    img.convertTo(img, CV_32FC3, 1.0 / 255);

    std::vector<float> input(3 * _input_h * _input_w);
    std::vector<cv::Mat> channels;
    for (int c = 0; c < 3; ++c) {
        channels.emplace_back(_input_h, _input_w, CV_32F, input.data() + c * _input_h * _input_w);
    }
    cv::split(img, channels);

    std::vector<float> output(_num_dets * 6);
    cudaMemcpyAsync(_input_dev, input.data(), input.size() * sizeof(float), cudaMemcpyHostToDevice, _stream);
    _context->enqueueV3(_stream);
    cudaMemcpyAsync(output.data(), _output_dev, output.size() * sizeof(float), cudaMemcpyDeviceToHost, _stream);
    cudaStreamSynchronize(_stream);

    const float sx = static_cast<float>(bgr.cols) / _input_w;
    const float sy = static_cast<float>(bgr.rows) / _input_h;

    std::vector<Detection> detections;
    for (int i = 0; i < _num_dets; ++i) {
        const float* d = &output[i * 6]; // x1, y1, x2, y2, score, class
        if (d[4] < min_score) continue;
        cv::Rect box(cv::Point(cvRound(d[0] * sx), cvRound(d[1] * sy)),
                    cv::Point(cvRound(d[2] * sx), cvRound(d[3] * sy)));
        detections.push_back({box, static_cast<int>(d[5]), d[4]});
    }
    return detections;
}

