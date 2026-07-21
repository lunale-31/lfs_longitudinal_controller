#pragma once

// Libraries required
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <iostream> 
#include <cmath> 
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <algorithm>

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
        // Considering both actuator and tire limits 
        double ax_max_accln;  
        double ay_max;
        double ax_max_brake;

        double ax_max_actuator_accln;
        double ax_max_actuator_brake; 

        // MAX VELOCITY
        double v_max = 32.16; // Theoretical maximum 
        // double v_max = 15.0; 

        VehicleParams(){
            double F_drive = max_drive_torque / wheel_radius;
            double F_brake = max_brake_torque / wheel_radius;
            double F_rr = mass * g * Crr;

            // Not considering drag as the ax is maximum only when the velocity = 0 as the whole term decreases when v increases. 
            ax_max_actuator_accln = (F_drive - F_rr) / mass; 
            ax_max_accln = std::min(ax_max_actuator_accln, mu_long*g);

            ay_max = mu_lat*g;
            
            ax_max_actuator_brake = (F_brake + F_rr) / mass; 
            ax_max_brake = std::min(ax_max_actuator_brake, mu_long*g);
        }
    };

    // FAHH helper
    std::vector<double> computeSmoothVelFromCSV(
        const std::string& input_filename,
        const std::string& output_filename,
        const VehicleParams& config_,
        std::vector<double>& cumulative_s,
        std::vector<double>& spline_curvature,
        std::vector<double>& v_corner,
        std::vector<double>& v_accln,
        std::vector<double>& v_brake
    );

    // Velocity profile calculation
    std::vector<double> computeCurvature(const std::vector<geometry_msgs::msg::Point>& latest_track_center, const VehicleParams& config_);
    std::vector<double> computeCornerVelocity(const std::vector<double>& k, const VehicleParams& config_); 
    std::vector<double> computeDeltaS(const std::vector<Eigen::Vector2d>& spline_points_);

    // Velocity profile calculation for control callback
    std::vector<double> getCumulativeS(const std::vector<Eigen::Vector2d>& spline_points_); 

    // Smooth Velocity profile
    std::vector<double> computeSmoothVel(const std::vector<Eigen::Vector2d>& spline_points_, const std::vector<double>& spline_curvature_, 
                                        const VehicleParams& config_, double current_speed, 
                                        std::vector<double>& v_corner, std::vector<double>& v_accln, std::vector<double>& v_brake);
    
    // Feedforward computation
    double computeFeedforward(double target_acceleration, double target_velocity, const VehicleParams& config_); 
    
    // CSV save
    void saveProfileAnalysisToCSV(
        const std::vector<Eigen::Vector2d>& spline_points,
        const std::vector<double>& spline_curvature,
        const VehicleParams& config_,
        const std::vector<double>& v_profile,
        const std::vector<double>& v_accln,
        const std::vector<double>& v_brake,
        const std::vector<double>& v_corner,
        const std::string& filename); 
    // Read-only 
    // void getK() const; 
    // void getVelProfile() const;

    /*TODO: 
    1) Create a function getCummulativeS() which computes the cummulative arc length s (as of now, we have delta S which says
    distance between point P0 and P1, P1 and P2... so on. But we need full arc length and points on the spline that represents the track points P0,P1...)
    2) For every controller callback, take s and fmod it with track length to obtain the current progress in spline (even if it completes multiple laps).
    3) Find the closest track point to that current progress
    4) Use that point index to take v_profile[idx] for the current instant.
    5) Better way is to linearly interpolate between 2 track points, instead of choosing 1 closest point as index. */

}