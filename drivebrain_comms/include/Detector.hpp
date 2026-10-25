#pragma once

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/core.hpp>

#include <memory>  
#include <string>
#include <vector>

namespace comms {

struct Detection { int class_id; float score; };

class Detector {
    public: 
        /**
         * @brief Construct a new Detector object
         * 
         * @param engine_path Path to the TensorRT engine file
         */
        Detector(const std::string& engine_path);

        ~Detector();

        /**
         * @brief Classify the colour of each cone patch
         * 
         * @param patches RGB cone patches (CV_8UC3) at the engine's input size
         * @return std::vector<Detection> Most likely class and its softmax probability per patch, up to the engine's max batch
         */
        std::vector<Detection> classify(const std::vector<cv::Mat>& patches);

    private: 
        std::unique_ptr<nvinfer1::IRuntime> _runtime;
        std::unique_ptr<nvinfer1::ICudaEngine> _engine;
        std::unique_ptr<nvinfer1::IExecutionContext> _context;
        cudaStream_t _stream{nullptr};
        void* _input_dev{nullptr};
        void* _output_dev{nullptr};
        const char* _input_name{nullptr};
        int _input_w{0}, _input_h{0}, _max_batch{0}, _num_classes{0};
};
} // namespace comms