std_msgs/Header header

float64 u # Final Control signal 
float64 u_ff #  Feedforward signal
float64 u_pid # PID signal 
float64 u_unsaturated # Unsaturated u (before windup)
 
float64 u_p # Individual gains 
float64 u_i 
float64 u_d 

float64 v_target # Target velocity
float64 xdot # Current velocity

float64 a_target # Target acceleration

float64 performance_fraction # Performance fraction 

