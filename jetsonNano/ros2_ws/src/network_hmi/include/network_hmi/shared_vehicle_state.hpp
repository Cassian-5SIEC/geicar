#pragma once

#include <mutex>

// Thread-safe class to hold the robot's command state
class SharedVehicleState
{
public:
    struct State {
        int mode = 0;
        bool start = false;
    };

    void set_mode(int mode);
    void set_start(bool start);
    
    // Stops the vehicle unless it's in autonomous mode
    void stop_if_not_autonomous();
    void emergency_stop();
    State get_state();

private:
    std::mutex mutex_;
    State current_state_;
};