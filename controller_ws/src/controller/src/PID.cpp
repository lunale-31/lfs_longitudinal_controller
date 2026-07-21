#include "PID.h"

PID::PID(float kp, float ti, float td, float tr, float n, float beta, float dt): 
    m_kp{kp}, m_ti{ti}, m_td{td}, m_tr{tr}, m_n{n}, m_beta{beta}, m_dt{dt}
{
    ad = m_td/(m_td + m_n*dt);
    bd = m_kp*ad*m_n; 
} 

PID::PID(): m_kp(0), m_ti(0), m_td(0), m_tr(0), m_n(0), m_beta(0), m_dt(0) {}

double PID::calculateOutput(const double current_meas, double setpoint, double u_ff){
    // Error
    err = setpoint - current_meas; 
    
    // Proportional
    m_p = m_kp*(m_beta*setpoint - current_meas);

    // Derivative
    m_d = ad*prev_der - bd*(current_meas - prev_meas);   

    // PID output 
    m_pid = m_p + m_d + m_i;  
    m_v = m_p + m_d + m_i + u_ff; 

    // Integral back-calculation 
    m_u = m_v;
    
    if(m_u>MAX_THROTTLE){
        m_u = MAX_THROTTLE; 
    }
    if(m_u<MIN_THROTTLE){
        m_u = MIN_THROTTLE; 
    }
    
    // PID integral division by 0 states check
    double integral_step{}, tracking_step{}; 
    
    if(std::abs(m_ti)>1e-6){
        integral_step =  (m_kp*m_dt*err)/ m_ti; 
    }

    if(std::abs(m_tr)>1e-6){
        tracking_step = m_dt*(m_u - m_v)/m_tr; 
    }
    m_i = m_i + integral_step + tracking_step; 

    // Updates
    prev_meas = current_meas;
    prev_der = m_d; 

    return m_u; 
}

void PID::resetController(){
    m_i = 0.0;
    m_d = 0.0;
    prev_meas = 0.0; 
}

double PID::getUnsaturatedOutput() const{
    return m_v; 
}

double PID::getPIDoutput() const{
    return m_pid; 
}