#pragma once

// Headers
#include "PID.h"
#include "utils.h"

// Libraries required
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <iostream> 
#include <cmath> 
#include <chrono>

using namespace std::chrono_literals;

// Spline required to interpolate the track points
#include "spline_library/splines/uniform_cubic_bspline.h"
#include "spline_library/utils/arclength.h"
#include "spline_library/utils/splineinverter.h"

// Service to obtain the track points 
#include "track_srv/srv/return_track.hpp"

// Messages required
#include "lfs_msgs/msg/bike_state.hpp"
#include "lfs_msgs/msg/control_debug.hpp"
#include "lfs_msgs/msg/profile_debug.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"

#include <fstream>

class Controller : public rclcpp::Node{

    public: 
        Controller();

    private: 
        // Load parameters from the config yaml file 
        void loadParams();

        // Service request function
        void trackServiceRequest(); 

        // Controller loop callback after subscribing
        void controllerCallback(const lfs_msgs::msg::BikeState::SharedPtr msg);

        // Initialization of publisher and subscriber
        rclcpp::Subscription<lfs_msgs::msg::BikeState>::SharedPtr state_subscriber;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_publisher; 
        
        rclcpp::Publisher<lfs_msgs::msg::ControlDebug>::SharedPtr control_debug_pub; 
        rclcpp::Publisher<lfs_msgs::msg::ProfileDebug>::SharedPtr profile_debug_pub;
        
        // Create a client to request the track center points from return-track service 
        rclcpp::Client<track_srv::srv::ReturnTrack>::SharedPtr track_client; 

        // Initialization of variables
        std::vector<geometry_msgs::msg::Point> latest_track_center; // service response is a vector of points, check return_track.cpp file for verifying. 
        bool has_received_track = false;

        // To fit Bspline on the obtained points
        std::vector<Eigen::Vector2d> spline_control_points; // To convert geometry_msg points to Eigen::Vector2d 
        std::unique_ptr<LoopingUniformCubicBSpline<Eigen::Vector2d>> spline_; // Pointing towards the Bspline Object that contains utility functions
        
        std::vector<Eigen::Vector2d> spline_points_; 
        std::vector<double> spline_curvature_; 

        // Velocity profile plots
        utils::VehicleParams config_;
        std::vector<double> v_accln;
        std::vector<double> v_brake;
        std::vector<double> v_corner;
        std::vector<double> v_profile;
        
        // Velocity profile variables
        std::vector<double> s; 
        double track_length; 

        std::ofstream curvature_file;

        // PID and Params variables
        PID pid_1; 
        float kp, ti, td, tr, n, beta, dt;
}; 