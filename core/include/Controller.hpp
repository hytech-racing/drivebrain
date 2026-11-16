#pragma once

/**
 * This class represents a singler controller that operates
 * at a user-specified frequency and can be stepped at said frequency
 * to go from a controller input to a controller result
 */
namespace control {
    
    template <typename ControllerResult, typename ControllerInput>
    class Controller {
    public:
        Controller() = default;
        
        /**
         * @brief get the desired time delta / discrete time in seconds of the controller
         * @return time delta in seconds
         */
        virtual float get_dt_sec() = 0;

        /**
         * @brief step the controler
         * @param in the input of the controller
         * @return the result of the controller
         */
        virtual ControllerResult step_controller(const ControllerInput& in) = 0;
    };
}
