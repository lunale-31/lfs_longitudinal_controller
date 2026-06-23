#pragma once

// Libraries required
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <iostream> 
#include <cmath> 

// Messages required
#include "lfs_msgs/msg/bike_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"

class Controller : public rclcpp::Node{

    public: 
        Controller();

    private: 
        // Load parameters
        // void loadParams();

        // Controller loop callback after subscribing
        void controllerCallback(const lfs_msgs::msg::BikeState::SharedPtr msg);
        
        // Track center point callback
        // void trackCallback(const geometry_msgs::msg::Point::SharedPtr msg);

        // Initialization of publisher and subscriber
        rclcpp::Subscription<lfs_msgs::msg::BikeState>::SharedPtr state_subscriber;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_publisher; 
        //TODO: Learn service and request in ros2 and find a way to get the data from a service for track_center points. 
        // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr track_subscriber; 

        // Initialization of variables
        // geometry_msgs::msg::Point latest_track_center;
        // bool has_received_track = false;
}; 