#pragma once

#include <cmath>
#include <foxglove/FrameTransform.pb.h>
#include <limits>
#include <random>
#include <vector>
#include <algorithm>
#include <numbers>
#include <StateTracker.hpp>
#include "base_msgs.pb.h"
#include "dv_msgs.pb.h"
#include "hytech_msgs.pb.h"
#include <delaunator.hpp>
#include <FoxgloveServer.hpp>



namespace planning {

    /**
      Plans a drivable path through a known set of cones
      
      @param cones The cones to plan through, map frame
      @return The planned path, map frame
    */  
    std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones); 


} // planning namespace
