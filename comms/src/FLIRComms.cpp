#include "FLIRComms.hpp"
#include "FoxgloveServer.hpp"
#include "Telemetry.hpp"
#include "YOLODetection.hpp"

#include <NvInfer.h>
#include <NvInferVersion.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <arv.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <foxglove/CompressedImage.pb.h>
#include <foxglove/ImageAnnotations.pb.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if NV_TENSORRT_MAJOR != 10
#error "FLIRDriver requires TensorRT 10.x"
#endif

namespace comms {
namespace {

void check_cuda(cudaError_t status) {
    if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}

void check_aravis(GError*& error) {
    if (!error) return;
    std::string message = error->message;
    g_clear_error(&error);
    throw std::runtime_error(message);
}

class TRTLogger final : public nvinfer1::ILogger {
    void log(Severity severity, const char* message) noexcept override {
        try {
            if (severity <= Severity::kERROR) spdlog::error("TensorRT: {}", message);
            else if (severity == Severity::kWARNING) spdlog::warn("TensorRT: {}", message);
        } catch (...) {}
    }
};

cv::Mat image_from_buffer(ArvBuffer* buffer) {
    const int width = arv_buffer_get_image_width(buffer);
    const int height = arv_buffer_get_image_height(buffer);
    int x_padding = 0, y_padding = 0;
    arv_buffer_get_image_padding(buffer, &x_padding, &y_padding);
    const auto format = arv_buffer_get_image_pixel_format(buffer);
    int channels = 1;
    int conversion = -1;
    switch (format) {
        case ARV_PIXEL_FORMAT_MONO_8: conversion = cv::COLOR_GRAY2BGR; break;
        case ARV_PIXEL_FORMAT_BAYER_RG_8: conversion = cv::COLOR_BayerRGGB2BGR; break;
        case ARV_PIXEL_FORMAT_BAYER_BG_8: conversion = cv::COLOR_BayerBGGR2BGR; break;
        case ARV_PIXEL_FORMAT_BAYER_GR_8: conversion = cv::COLOR_BayerGRBG2BGR; break;
        case ARV_PIXEL_FORMAT_BAYER_GB_8: conversion = cv::COLOR_BayerGBRG2BGR; break;
        case ARV_PIXEL_FORMAT_RGB_8_PACKED: channels = 3; conversion = cv::COLOR_RGB2BGR; break;
        case ARV_PIXEL_FORMAT_BGR_8_PACKED: channels = 3; break;
        default: throw std::runtime_error("Unsupported camera pixel format; select Mono8, BayerXX8, RGB8 or BGR8");
    }
    if (width <= 0 || height <= 0 || x_padding < 0)
        throw std::runtime_error("Invalid camera image dimensions");
    size_t size = 0;
    const void* data = arv_buffer_get_image_data(buffer, &size);
    const size_t stride = static_cast<size_t>(width) * channels + x_padding;
    if (!data || size / stride < static_cast<size_t>(height))
        throw std::runtime_error("Truncated camera frame");
    cv::Mat view(height, width, CV_MAKETYPE(CV_8U, channels), const_cast<void*>(data), stride);
    cv::Mat bgr;
    if (conversion < 0) bgr = view.clone();
    else cv::cvtColor(view, bgr, conversion);
    return bgr;
}

void stamp(google::protobuf::Timestamp* timestamp, int64_t ns) {
    timestamp->set_seconds(ns / 1000000000);
    timestamp->set_nanos(static_cast<int32_t>(ns % 1000000000));
}

}

struct FLIRDriver::Impl {
    TRTLogger logger;
    std::unique_ptr<nvinfer1::IRuntime> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine> engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    cudaStream_t cuda_stream = nullptr;
    void* input_device = nullptr;
    void* output_device = nullptr;
    std::vector<float> input, output;
    int input_width = 0, input_height = 0;
    float confidence = 0.25f;
    int jpeg_quality = 80;
    std::string frame_id;
    ArvCamera* camera = nullptr;
    ArvStream* stream = nullptr;
    bool acquiring = false;
    std::atomic<bool> running{false};
    std::thread worker;
    uint64_t discarded = 0, invalid = 0;

    ~Impl() {
        running = false;
        if (worker.joinable()) worker.join();
        if (acquiring) {
            GError* error = nullptr;
            arv_camera_stop_acquisition(camera, &error);
            if (error) { spdlog::warn("FLIR stop: {}", error->message); g_clear_error(&error); }
        }
        if (stream) g_object_unref(stream);
        if (camera) g_object_unref(camera);
        if (cuda_stream) cudaStreamSynchronize(cuda_stream);
        context.reset();
        if (input_device) cudaFree(input_device);
        if (output_device) cudaFree(output_device);
        if (cuda_stream) cudaStreamDestroy(cuda_stream);
    }

    void load_model(const std::string& path, const std::string& device, bool fallback, bool fp16) {
        check_cuda(cudaSetDevice(0));
        std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(logger));
        if (!builder) throw std::runtime_error("Could not create TensorRT builder");
        std::unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(0));
        std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
        if (!network || !config) throw std::runtime_error("Could not create TensorRT network/config");
        std::unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*network, logger));
        if (!parser || !parser->parseFromFile(path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
            throw std::runtime_error("Could not parse ONNX model: " + path);
        if (network->getNbInputs() != 1 || network->getNbOutputs() != 1)
            throw std::runtime_error("Expected one YOLO26 image input and one detection output");
        auto* in = network->getInput(0);
        auto* out = network->getOutput(0);
        const auto dims = in->getDimensions();
        if (dims.nbDims != 4 || dims.d[0] != 1 || dims.d[1] != 3 || dims.d[2] <= 0 || dims.d[3] <= 0 ||
            dims.d[2] > 8192 || dims.d[3] > 8192 || in->getType() != nvinfer1::DataType::kFLOAT)
            throw std::runtime_error("Export ONNX with fixed FP32 input [1,3,H,W], batch=1, dynamic=False");
        const auto output_dims = out->getDimensions();
        if (output_dims.nbDims != 3 || output_dims.d[0] != 1 || output_dims.d[1] <= 0 ||
            output_dims.d[1] > 10000 || output_dims.d[2] != 6 || out->getType() != nvinfer1::DataType::kFLOAT)
            throw std::runtime_error("Expected YOLO26 end-to-end FP32 output [1,N,6]: xyxy, confidence, class");
        input_height = static_cast<int>(dims.d[2]);
        input_width = static_cast<int>(dims.d[3]);
        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30);
        if (fp16) config->setFlag(nvinfer1::BuilderFlag::kFP16);
        if (device != "gpu") {
            const int core = device == "dla0" ? 0 : device == "dla1" ? 1 : -1;
            if (core < 0 || core >= builder->getNbDLACores() || !fp16)
                throw std::runtime_error("Select gpu, or available dla0/dla1 with fp16=true");
            config->setDefaultDeviceType(nvinfer1::DeviceType::kDLA);
            config->setDLACore(core);
            if (fallback) config->setFlag(nvinfer1::BuilderFlag::kGPU_FALLBACK);
        }
        spdlog::info("Building FLIR TensorRT engine: {}, device={}, GPU fallback={}", path, device, fallback);
        std::unique_ptr<nvinfer1::IHostMemory> plan(builder->buildSerializedNetwork(*network, *config));
        if (!plan) throw std::runtime_error("TensorRT engine build failed; see layer/DLA diagnostics above");
        runtime.reset(nvinfer1::createInferRuntime(logger));
        if (!runtime) throw std::runtime_error("Could not create TensorRT runtime");
        if (device != "gpu") runtime->setDLACore(device == "dla0" ? 0 : 1);
        engine.reset(runtime->deserializeCudaEngine(plan->data(), plan->size()));
        if (!engine) throw std::runtime_error("Could not deserialize TensorRT engine");
        context.reset(engine->createExecutionContext());
        if (!context) throw std::runtime_error("Could not create TensorRT execution context");
        input.resize(static_cast<size_t>(input_width) * input_height * 3);
        output.resize(static_cast<size_t>(output_dims.d[1]) * 6);
        check_cuda(cudaStreamCreateWithFlags(&cuda_stream, cudaStreamNonBlocking));
        check_cuda(cudaMalloc(&input_device, input.size() * sizeof(float)));
        check_cuda(cudaMalloc(&output_device, output.size() * sizeof(float)));
        for (int i = 0; i < engine->getNbIOTensors(); ++i) {
            const char* name = engine->getIOTensorName(i);
            if (engine->getTensorDataType(name) != nvinfer1::DataType::kFLOAT ||
                engine->getTensorFormat(name) != nvinfer1::TensorFormat::kLINEAR ||
                engine->getTensorLocation(name) != nvinfer1::TensorLocation::kDEVICE)
                throw std::runtime_error("TensorRT engine requires unsupported I/O layout");
            if (!context->setTensorAddress(name, engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT
                                               ? input_device : output_device))
                throw std::runtime_error("Could not bind TensorRT tensor");
        }
    }

    void open_camera(const std::string& id, const std::string& pixel_format, double fps) {
        GError* error = nullptr;
        camera = arv_camera_new(id.c_str(), &error);
        check_aravis(error);
        if (!camera || !arv_camera_is_gv_device(camera)) throw std::runtime_error("Camera is not a GigE Vision device");
        arv_camera_clear_triggers(camera, &error);
        check_aravis(error);
        arv_camera_set_pixel_format_from_string(camera, pixel_format.c_str(), &error);
        check_aravis(error);
        arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS, &error);
        check_aravis(error);
        arv_camera_set_frame_rate(camera, fps, &error);
        check_aravis(error);
        const auto payload = arv_camera_get_payload(camera, &error);
        check_aravis(error);
        if (!payload) throw std::runtime_error("Camera payload is empty");
        stream = arv_camera_create_stream(camera, nullptr, nullptr, &error);
        check_aravis(error);
        if (!stream) throw std::runtime_error("Could not create Aravis stream");
        for (int i = 0; i < 8; ++i) {
            auto* buffer = arv_buffer_new_allocate(payload);
            if (!buffer) throw std::runtime_error("Could not allocate camera buffer");
            arv_stream_push_buffer(stream, buffer);
        }
        arv_camera_start_acquisition(camera, &error);
        check_aravis(error);
        acquiring = true;
    }

    void process(const cv::Mat& bgr, int64_t timestamp) {
        const auto transform = yolo::letterbox(bgr.cols, bgr.rows, input_width, input_height);
        cv::Mat resized, padded(input_height, input_width, CV_8UC3, cv::Scalar(114, 114, 114));
        cv::resize(bgr, resized, cv::Size(transform.width, transform.height));
        resized.copyTo(padded(cv::Rect(transform.left, transform.top, transform.width, transform.height)));
        const size_t plane = static_cast<size_t>(input_width) * input_height;
        for (int y = 0; y < input_height; ++y) {
            const auto* row = padded.ptr<cv::Vec3b>(y);
            for (int x = 0; x < input_width; ++x) {
                const auto index = static_cast<size_t>(y) * input_width + x;
                input[index] = row[x][2] / 255.0f;
                input[plane + index] = row[x][1] / 255.0f;
                input[2 * plane + index] = row[x][0] / 255.0f;
            }
        }
        check_cuda(cudaMemcpyAsync(input_device, input.data(), input.size() * sizeof(float), cudaMemcpyHostToDevice, cuda_stream));
        if (!context->enqueueV3(cuda_stream)) throw std::runtime_error("TensorRT enqueue failed");
        check_cuda(cudaMemcpyAsync(output.data(), output_device, output.size() * sizeof(float), cudaMemcpyDeviceToHost, cuda_stream));
        check_cuda(cudaStreamSynchronize(cuda_stream));
        auto annotations = std::make_shared<foxglove::ImageAnnotations>();
        for (size_t i = 0; i < output.size(); i += 6) {
            auto detection = yolo::decode(output.data() + i, confidence, transform, bgr.cols, bgr.rows);
            if (!detection) continue;
            const auto& d = *detection;
            auto* box = annotations->add_points();
            stamp(box->mutable_timestamp(), timestamp);
            box->set_type(foxglove::PointsAnnotation::LINE_LOOP);
            box->set_thickness(2);
            box->mutable_outline_color()->set_g(1);
            box->mutable_outline_color()->set_a(1);
            for (auto point : {cv::Point2f(d.x1, d.y1), cv::Point2f(d.x2, d.y1),
                               cv::Point2f(d.x2, d.y2), cv::Point2f(d.x1, d.y2)}) {
                auto* p = box->add_points(); p->set_x(point.x); p->set_y(point.y);
            }
            auto* text = annotations->add_texts();
            stamp(text->mutable_timestamp(), timestamp);
            text->mutable_position()->set_x(d.x1);
            text->mutable_position()->set_y(d.y1);
            text->set_text(fmt::format("{} {:.2f}", d.class_id, d.confidence));
            text->set_font_size(16);
            text->mutable_text_color()->set_g(1);
            text->mutable_text_color()->set_a(1);
        }
        std::vector<unsigned char> jpeg;
        if (!cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality}))
            throw std::runtime_error("JPEG encoding failed");
        auto image = std::make_shared<foxglove::CompressedImage>();
        stamp(image->mutable_timestamp(), timestamp);
        image->set_frame_id(frame_id);
        image->set_format("jpeg");
        image->set_data(jpeg.data(), jpeg.size());
        core::log(image);
        core::log(annotations);
    }

    void receive() {
        try {
            check_cuda(cudaSetDevice(0));
            auto report = std::chrono::steady_clock::now();
            while (running) {
                ArvBuffer* buffer = arv_stream_timeout_pop_buffer(stream, 100000);
                if (!buffer) continue;
                for (int i = 0; i < 7; ++i) {
                    ArvBuffer* next = arv_stream_try_pop_buffer(stream);
                    if (!next) break;
                    if (arv_buffer_get_status(next) != ARV_BUFFER_STATUS_SUCCESS) {
                        ++invalid;
                        arv_stream_push_buffer(stream, next);
                    } else {
                        ++discarded;
                        arv_stream_push_buffer(stream, buffer);
                        buffer = next;
                    }
                }
                if (arv_buffer_get_status(buffer) != ARV_BUFFER_STATUS_SUCCESS) {
                    ++invalid;
                    arv_stream_push_buffer(stream, buffer);
                    continue;
                }
                const int64_t timestamp = static_cast<int64_t>(arv_buffer_get_system_timestamp(buffer));
                cv::Mat bgr;
                try { bgr = image_from_buffer(buffer); }
                catch (...) { arv_stream_push_buffer(stream, buffer); throw; }
                arv_stream_push_buffer(stream, buffer);
                if (!running) break;
                const auto start = std::chrono::steady_clock::now();
                process(bgr, timestamp);
                if (start - report >= std::chrono::seconds(5)) {
                    spdlog::info("FLIR processing {:.1f} ms, discarded={}, invalid={}",
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count(), discarded, invalid);
                    report = start;
                }
            }
        } catch (const std::exception& e) {
            spdlog::error("FLIR stream stopped: {}", e.what());
        }
        running = false;
    }
};

FLIRDriver::FLIRDriver(bool& init_not_successful) {
    init_not_successful = !init();
}

FLIRDriver::~FLIRDriver() { stop(); }

bool FLIRDriver::init() {
    if (_impl) return _impl->running;
    try {
        auto& params = core::FoxgloveServer::instance();
        auto model = params.get_param<std::string>("flir_driver/model_path");
        auto camera = params.get_param<std::string>("flir_driver/device_name");
        if (!model || model->empty() || !camera || camera->empty())
            throw std::runtime_error("Set flir_driver/model_path and device_name");
        auto state = std::make_unique<Impl>();
        state->confidence = params.get_param<double>("flir_driver/confidence").value_or(0.25);
        state->jpeg_quality = params.get_param<int>("flir_driver/jpeg_quality").value_or(80);
        state->frame_id = params.get_param<std::string>("flir_driver/frame_id").value_or("flir_camera");
        const double fps = params.get_param<double>("flir_driver/fps").value_or(15.0);
        if (!std::isfinite(state->confidence) || state->confidence < 0 || state->confidence > 1 ||
            state->jpeg_quality < 1 || state->jpeg_quality > 100 || !std::isfinite(fps) || fps <= 0)
            throw std::runtime_error("Invalid FLIR confidence, JPEG quality or frame rate");
        state->load_model(*model, params.get_param<std::string>("flir_driver/device").value_or("gpu"),
                          params.get_param<bool>("flir_driver/gpu_fallback").value_or(false),
                          params.get_param<bool>("flir_driver/fp16").value_or(true));
        state->open_camera(*camera, params.get_param<std::string>("flir_driver/pixel_format").value_or("BayerRG8"), fps);
        state->running = true;
        state->worker = std::thread(&Impl::receive, state.get());
        _impl = std::move(state);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("FLIR initialization failed: {}", e.what());
        return false;
    }
}

void FLIRDriver::stop() { _impl.reset(); }

}
