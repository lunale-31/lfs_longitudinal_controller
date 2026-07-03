#pragma once

// Headers
#include "PID.h"

// Libraries required
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <iostream> 
#include <cmath> 
#include <chrono>

using namespace std::chrono_literals;

// Service
#include "track_srv/srv/return_track.hpp"

// Messages required
#include "lfs_msgs/msg/bike_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"

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
        
        // Create a client to request the track center points from return-track service 
        rclcpp::Client<track_srv::srv::ReturnTrack>::SharedPtr track_client; 

        // Initialization of variables
        std::vector<geometry_msgs::msg::Point> latest_track_center; // service response is a vector of points, check return_track.cpp file for verifying. 
        bool has_received_track = false;

        // Velocity profile plots
        std::vector<double> v_accln;
        std::vector<double> v_brake;
        std::vector<double> v_corner;
        std::vector<double> v_profile;
        
        // PID and Params variables
        PID pid_1; 
        float kp, ti, td, tr, n, beta, dt;
}; 