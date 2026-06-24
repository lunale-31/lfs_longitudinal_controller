#include "long_controller.h"

Controller::Controller() : rclcpp::Node("longitudinal_controller"){

    state_subscriber = this->create_subscription<lfs_msgs::msg::BikeState>("/current_state", 10, std::bind(&Controller::controllerCallback, this, std::placeholders::_1)); 
    throttle_publisher = this->create_publisher<std_msgs::msg::Float64>("/throttle_cmd", 10); 

    track_client = this->create_client<track_srv::srv::ReturnTrack>("/return_track");  // Enter the service name correctly when you create a client.
    
    RCLCPP_INFO(this->get_logger(), "Controller is active! waiting for simulation ticks..."); 
} 

void Controller::trackServiceRequest(){

    // Wait for service to become available
    while(!track_client->wait_for_service(1s)){
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }    
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // Create a request message
    auto request = std::make_shared<track_srv::srv::ReturnTrack::Request>();

    // Get the response using asychronous callback to prevent controller callback loop freeze. 
    track_client->async_send_request(request,
        [this](rclcpp::Client<track_srv::srv::ReturnTrack>::SharedFuture future) {
            auto response = future.get();
            
            // Handle the response data 
            this->latest_track_center = response->points; 
            this->has_received_track = true;
            
            RCLCPP_INFO(this->get_logger(), "Successfully received track center point!");
        }); 

}

void Controller::controllerCallback(const lfs_msgs::msg::BikeState::SharedPtr msg){
    
    this->trackServiceRequest();

    // Check if the controller callback has received the track center points
    if (!has_received_track) {
        RCLCPP_WARN(this->get_logger(), "Waiting for first track center points...");
        return;
    }

    // Check for track center point service response 
    RCLCPP_INFO(this->get_logger(), "Received %zu track center points!", this->latest_track_center.size());

    if (!this->latest_track_center.empty()) {
        RCLCPP_INFO(this->get_logger(), "First point -> X: %f, Y: %f, Z: %f", 
                    this->latest_track_center[0].x, 
                    this->latest_track_center[0].y, 
                    this->latest_track_center[0].z);
    } else {
        RCLCPP_WARN(this->get_logger(), "The received track points array is empty!");
    }
    // RCLCPP_INFO(this->get_logger(), "Received Bike State! Current speed: %f", msg->x_dot);
}