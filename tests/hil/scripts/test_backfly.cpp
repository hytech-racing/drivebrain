#include <arv.h>
#include <iostream>
#include <atomic>
#include <thread>

// Flag to gracefully control the frame acquisition loop
std::atomic<bool> g_running(true);

int main() {
    ArvCamera *camera = nullptr;
    ArvStream *stream = nullptr;
    GError *error = nullptr;

    // 1. Connect to the first available camera
    camera = arv_camera_new(nullptr, &error);
    if (!ARV_IS_CAMERA(camera)) {
        std::cerr << "Failed to detect camera: " 
                  << (error ? error->message : "Unknown error") << std::endl;
        g_clear_error(&error);
        return -1;
    }
    std::cout << "Connected to: " << arv_camera_get_model_name(camera, nullptr) << std::endl;

    // 2. Set the acquisition mode to Continuous 
    arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS, &error);
    
    // 3. Retrieve image payload (size of one raw frame in bytes)
    gint payload = arv_camera_get_payload(camera, &error);
    
    // 4. Create the stream object
    stream = arv_camera_create_stream(camera, nullptr, nullptr, &error);
    if (!ARV_IS_STREAM(stream)) {
        std::cerr << "Failed to create stream framework." << std::endl;
        g_object_unref(camera);
        return -1;
    }

    // 5. Pre-allocate and push buffers into the stream queue pool
    int buffer_pool_size = 20; 
    for (int i = 0; i < buffer_pool_size; i++) {
        arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));
    }

    // 6. Start the physical camera acquisition hardware
    arv_camera_start_acquisition(camera, &error);
    std::cout << "Acquisition started successfully. Capturing continuous frames..." << std::endl;

    // 7. Continuous Pop-and-Push Loop
    while (g_running) {
        ArvBuffer *buffer = nullptr;

        // Pop the oldest available filled frame buffer (timeout in microseconds: 1,000,000us = 1s)
        buffer = arv_stream_timeout_pop_buffer(stream, 1000000); 

        if (buffer != nullptr) {
            // Verify the frame status is successful before inspecting content
            if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
                size_t buffer_size;
                // Get the raw pointer to pixel data
                const void* raw_data = arv_buffer_get_data(buffer, &buffer_size);
                
                // --- CUSTOM DATA PROCESSING PLACEHOLDER ---
                // Example: Wrap data with OpenCV or process directly
                // cv::Mat frame(height, width, CV_8UC1, (void*)raw_data);
                std::cout << "Frame captured successfully! Bytes payload: " << buffer_size << std::endl;
            } else {
                std::cerr << "Frame packet dropped or incomplete buffer received." << std::endl;
            }

            // CRITICAL: Push the buffer back into the stream queue pool to prevent resource starvation
            arv_stream_push_buffer(stream, buffer);
        } else {
            std::cerr << "Timeout reached: No buffer received from camera stream." << std::endl;
        }
    }

    // 8. Proper Shutdown Sequence
    std::cout << "Stopping acquisition..." << std::endl;
    arv_camera_stop_acquisition(camera, nullptr);

    // Clean up allocated objects
    g_object_unref(stream);
    g_object_unref(camera);

    return 0;
}
