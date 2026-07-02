#pragma once

// Libraries required
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <iostream> 
#include <cmath> 

// Messages required
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"

namespace utils{
    /* msg->x_dot (to obtain states)
    1) latest_track_center has the 98 track_center points as a vector from the service. (latest_track_center[0].x, y, z). 
    2) Compute the track curvature (k).
    3) Use the k to calculate maximum speed the car can safely drive around a corner (lateral accln is needed to stay on the curve, 
    which is a centripetal accln) and driving on the straight line.
    4) Modify the obtained profile due to k using accln and braking limits. 
    5) Use this smooth profile to track using PID, which will generate throttle commands. 
    */

    /*
    TODO first: Think about mapping the indices of spline progression along with the respective velocity. 
    */
    struct VehicleParams{
        double   mass = 200.0;
        double   lf = 0.781;
        double   lr = 0.736;
        double   Iz = 260.0;
        double   mu_long = 0.8;
        double   mu_lat = 1.63;
        double   g = 9.81;
        double   wheel_radius = 0.19;
        double   max_drive_torque = 150.0;
        double   max_brake_torque = 200.0;
        double   Crr = 0.015;
        double   rho_air = 1.225;
        double   CdA = 1.2;
        double   dt = 0.005;
        
        // MAX ACCELERATION AND BRAKING
        // MAX ACCELERATION ON CORNERING
        double ax_max_accln;  
        double ay_max;
        double ax_max_brake;

        // MAX VELOCITY
        double v_max = 50.0; 

        VehicleParams(){
            double F_drive = max_drive_torque / wheel_radius;
            double F_rr = mass * g * Crr;
            ax_max_accln = (F_drive - F_rr) / mass; // Not considering drag as the ax is maximum only when the velocity = 0 as the whole term decreases when v increases. 
            ay_max = mu_lat*g;
            ax_max_brake = mu_long*g;
        }
    };

    // Velocity profile calculation
    std::vector<double> computeCurvature(const std::vector<geometry_msgs::msg::Point>& latest_track_center, const VehicleParams& config_);
    std::vector<double> computeVelocity(const std::vector<double>& k, const VehicleParams& config_); 
    
    // Smooth Velocity profile
    // std::vector<double> computeSmoothVel(const std::vector<geometry_msgs::msg::Point>& latest_track_center, const VehicleParams& config_);

    // Read-only 
    // void getK() const; 
    // void getVelProfile() const;
}