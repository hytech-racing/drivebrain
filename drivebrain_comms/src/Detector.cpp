#include "Detector.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
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

    if (_engine->getNbIOTensors() != 2) {
      throw std::runtime_error("Engine must have exactly one input and one output: " + engine_path);
    }
    _input_name = _engine->getIOTensorName(0);
    const char* output_name = _engine->getIOTensorName(1);
    const auto max_shape = _engine->getProfileShape(_input_name, 0, nvinfer1::OptProfileSelector::kMAX);
    _max_batch = max_shape.d[0];
    _input_h = max_shape.d[2];
    _input_w = max_shape.d[3];
    const auto out_shape = _engine->getTensorShape(output_name);
    if (out_shape.nbDims != 2) {
        throw std::runtime_error("Engine output must be [batch, classes]: " + engine_path);
    }
    _num_classes = out_shape.d[1];

    cudaStreamCreate(&_stream);
    cudaMalloc(&_input_dev, _max_batch * _input_h * _input_w * 3 * sizeof(float));
    cudaMalloc(&_output_dev, _max_batch * _num_classes * sizeof(float));

    _context->setTensorAddress(_input_name, _input_dev);
    _context->setTensorAddress(output_name, _output_dev);

    spdlog::info("Loaded {} ({}x{} input, batch up to {})", engine_path, _input_w, _input_h, _max_batch);
}

Detector::~Detector() {
    cudaFree(_input_dev);
    cudaFree(_output_dev);
    cudaStreamDestroy(_stream);
}

std::vector<Detection> Detector::classify(const std::vector<cv::Mat>& patches) {
    const int batch = std::min(static_cast<int>(patches.size()), _max_batch);
    if (batch == 0) return {};

    std::vector<float> input(batch * 3 * _input_h * _input_w);
    for (int i = 0; i < batch; ++i) {
        cv::Mat img;
        patches[i].convertTo(img, CV_32FC3, 1.0 / 255);

        std::vector<cv::Mat> channels;
        for (int c = 0; c < 3; ++c) {
            channels.emplace_back(_input_h, _input_w, CV_32F, input.data() + (i * 3 + c) * _input_h * _input_w);
        }
        cv::split(img, channels);
    }

    std::vector<float> output(batch * _num_classes);
    _context->setInputShape(_input_name, nvinfer1::Dims4{batch, 3, _input_h, _input_w});
    cudaMemcpyAsync(_input_dev, input.data(), input.size() * sizeof(float), cudaMemcpyHostToDevice, _stream);
    _context->enqueueV3(_stream);
    cudaMemcpyAsync(output.data(), _output_dev, output.size() * sizeof(float), cudaMemcpyDeviceToHost, _stream);
    cudaStreamSynchronize(_stream);

    std::vector<Detection> detections;
    for (int i = 0; i < batch; ++i) {
        const float* logits = &output[i * _num_classes]; // softmax of the most likely class
        const int best = std::max_element(logits, logits + _num_classes) - logits;
        float sum = 0.0f;
        for (int c = 0; c < _num_classes; ++c) sum += std::exp(logits[c] - logits[best]);
        detections.push_back({best, 1.0f / sum});
    }
    return detections;
}

