#pragma once

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/core.hpp>

#include <memory>  
#include <string>
#include <vector>

namespace comms {

struct Detection { cv::Rect bbox; int class_id; float score; };

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
         * @brief Detect cones in the given BGR image
         * 
         * @param bgr Input image in BGR format
         * @param min_score Minimum confidence score to consider a detection valid
         * @return std::vector<Detection> List of detected cones
         */
        std::vector<Detection> detect(const cv::Mat& bgr, float min_score);

        /**
         * @brief Get the name of the cone class corresponding to the given class ID
         * 
         * @param class_id Class ID of the cone
         * @return const char* Name of the cone class
         */
        inline const char* cone_name(int class_id) {
            static const char* names[] = {"blue", "yellow", "orange", "unknown"};
            return (class_id >= 0 && class_id < 4) ? names[class_id] : "invalid";
        }

        /**
         * @brief Get the color of the cone class corresponding to the given class ID
         * 
         * @param class_id Class ID of the cone
         * @return cv::Scalar Color of the cone class in BGR format
         */
        inline cv::Scalar cone_color(int class_id) { // BGR
            static const cv::Scalar colors[] = {{255, 0, 0}, {0, 255, 255}, {0, 165, 255}, {128, 128, 128}};
            return (class_id >= 0 && class_id < 4) ? colors[class_id] : cv::Scalar(255, 255, 255);
        }


    private: 
        std::unique_ptr<nvinfer1::IRuntime> _runtime;
        std::unique_ptr<nvinfer1::ICudaEngine> _engine;
        std::unique_ptr<nvinfer1::IExecutionContext> _context;
        cudaStream_t _stream{nullptr};
        void* _input_dev{nullptr};
        void* _output_dev{nullptr};
        int _input_w{0}, _input_h{0}, _num_dets{0};
};
} // namespace comms