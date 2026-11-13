#pragma once
#include <string>
#include <vector>

namespace core {
    inline std::vector<std::string> PROTO_NAMES = {"hytech_msgs.proto", "ht_can.proto", "dv_msgs.proto",
                                            "foxglove/PointCloud.proto", "foxglove/SceneUpdate.proto", "foxglove/FrameTransform.proto",
                                            "foxglove/CompressedVideo.proto", "foxglove/CompressedImage.proto", "foxglove/RawImage.proto", 
                                            "foxglove/ImageAnnotations.proto", "google/protobuf/struct.proto"};
}  // namespace core