#pragma once

#include <cmath>

class PID{
    private:
        // PID controller parameters
        float m_kp{}, m_ti{}, m_td{}, m_tr{}, m_n{}, m_beta{}, m_dt{};

        // PID controller update variables
        float m_v{}, m_u{}, m_p{}, m_i{}, m_d{}, ad{}, bd{}, err{}, prev_meas{}, prev_der{};

        static constexpr float MAX_THROTTLE = 1.0; // compile-time const, so no warnings
        static constexpr float MIN_THROTTLE = -1.0; // static = only 1 variable for every instances of the class

    public:
        // Constructor to initialize parameters    
        PID(float kp, float ti, float td, float tr, float n, float beta, float dt);
        PID(); 
        
        // Output calculation
        double calculateOutput(const double current_meas, double setpoint); 

        // Reset
        void resetController(); 
};