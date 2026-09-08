#pragma once

#include <memory>

#include "backend/IncrementalGraphSlamTypes.hpp"
#include "common/SlamInterfaces.hpp"
#include "dv_msgs.pb.h"

namespace adapters
{

std::shared_ptr<dv_msgs::SlamFrontendDebug> to_slam_frontend_debug(
    const slam::FrontendResult& result);

std::shared_ptr<dv_msgs::IncrementalGraphSlamDebug>
to_incremental_graph_slam_debug(
    const slam::LandmarkFrame& frame,
    const slam::backend::IncrementalGraphSlamResult& result);

}  // namespace adapters
