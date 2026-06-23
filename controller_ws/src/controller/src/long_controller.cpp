#include "long_controller.h"

Controller::Controller() : rclcpp::Node("longitudinal_controller"){

    state_subscriber = this->create_subscription<lfs_msgs::msg::BikeState>("/current_state", 10, std::bind(&Controller::controllerCallback, this, std::placeholders::_1)); 
    // track_subscriber = this->create_subscription<geometry_msgs::msg::Point>("/visualize_track", 10, std::bind(&Controller::trackCallback, this, std::placeholders::_1));

    throttle_publisher = this->create_publisher<std_msgs::msg::Float64>("/throttle_cmd", 10); 

    RCLCPP_INFO(this->get_logger(), "Controller is active! waiting for simulation ticks..."); 
} 

// void Controller::trackCallback(const geometry_msgs::msg::Point::SharedPtr msg){
//     latest_track_center = *msg;
//     has_received_track = true; 
// }

void Controller::controllerCallback(const lfs_msgs::msg::BikeState::SharedPtr msg){
    
    // if (!has_received_track) {
    //     RCLCPP_WARN(this->get_logger(), "Waiting for first track center points...");
    //     return;
    // }

    RCLCPP_INFO(this->get_logger(), "Received Bike State! Current speed: %f", msg->x_dot);
}