#pragma once

namespace estimation
{
    template <typename EstimatorResult, typename EstimatorInput>
    class Estimator
    {
        public:
            Estimator() = default;
            
            virtual float get_dt_sec() = 0;
            virtual EstimatorResult step_estimator(const EstimatorInput &in) = 0;
    };
}