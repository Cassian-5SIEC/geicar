#include "network_hmi/shared_vehicle_state.hpp"

void SharedVehicleState::set_mode(int mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_.mode = mode;
    // If we switch to autonomous, stop manual control
    if (current_state_.mode == 2) { 
        current_state_.start = false;
    }
}

void SharedVehicleState::set_start(bool start)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // Can only start if not in autonomous mode
    if (current_state_.mode != 2) {
        current_state_.start = start;
    }
}

void SharedVehicleState::stop_if_not_autonomous()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_state_.mode != 1) { // 1 is autonomous
        current_state_.start = false;
    }
}

void SharedVehicleState::emergency_stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_.start = false;
}

SharedVehicleState::State SharedVehicleState::get_state()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return current_state_;
}