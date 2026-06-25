#include "long_controller.h"
#include "PID.h"

Controller::Controller() : rclcpp::Node("longitudinal_controller"){

    state_subscriber = this->create_subscription<lfs_msgs::msg::BikeState>("/current_state", 10, std::bind(&Controller::controllerCallback, this, std::placeholders::_1)); 
    throttle_publisher = this->create_publisher<std_msgs::msg::Float64>("/throttle_cmd", 10); 

    track_client = this->create_client<track_srv::srv::ReturnTrack>("/return_track");  // Enter the service name correctly when you create a client.
    
    RCLCPP_INFO(this->get_logger(), "Controller is active! waiting for simulation ticks..."); 
    
    loadParams();
    pid_1 = PID(kp, ti, td, tr, n, beta, dt);

    this->trackServiceRequest(); // To collect the track points just one time 
} 

void Controller::loadParams(){
    // Default declaration for fail-safe 
    this->declare_parameter<double>("kp", 1.0);
    this->declare_parameter<double>("ti", 0.1);
    this->declare_parameter<double>("td", 0.01);
    this->declare_parameter<double>("tr", 0.01);
    this->declare_parameter<double>("n", 1.0);
    this->declare_parameter<double>("beta", 1.0);
    this->declare_parameter<double>("dt", 0.05);

    // Grab the values from yaml file
    this->get_parameter("kp", kp);
    this->get_parameter("ti", ti);
    this->get_parameter("td", td);
    this->get_parameter("tr", tr);
    this->get_parameter("n", n);
    this->get_parameter("beta", beta);
    this->get_parameter("dt", dt);
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

    // Check if the controller callback has received the track center points and then request for it 
    if (!has_received_track) {
        RCLCPP_WARN(this->get_logger(), "Waiting for first track center points...");
        return;
    }

    // // Check for track center point service response 
    // RCLCPP_INFO(this->get_logger(), "Received %zu track center points!", this->latest_track_center.size());

    // if (!this->latest_track_center.empty()) {
    //     RCLCPP_INFO(this->get_logger(), "First point -> X: %f, Y: %f, Z: %f", 
    //                 this->latest_track_center[0].x, 
    //                 this->latest_track_center[0].y, 
    //                 this->latest_track_center[0].z);
    // } else {
    //     RCLCPP_WARN(this->get_logger(), "The received track points array is empty!");
    // }
    // RCLCPP_INFO(this->get_logger(), "Received Bike State! Current speed: %f", msg->x_dot);

    // PID Controller
    float current_speed = msg->x_dot;
    float setpoint = 1.0;
    double u; 

    // Casting to double for PID calcs, the states are in float32 so. 
    u = pid_1.calculateOutput(static_cast<double>(current_speed), static_cast<double>(setpoint)); 

    std_msgs::msg::Float64 throttle_msg; 
    throttle_msg.data = u; 

    throttle_publisher->publish(throttle_msg); 
}