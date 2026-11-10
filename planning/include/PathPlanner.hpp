#pragma once

#include <vector>

#include <StateTracker.hpp>
#include "dv_msgs.pb.h"


namespace planning {

    /**
      Plans a drivable path through a known set of cones
      
      @param cones The cones to plan through, map frame
      @return The planned path, map frame
    */  
    std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones); 


} // planning namespace
