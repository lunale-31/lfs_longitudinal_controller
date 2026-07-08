#include "long_controller.h"
#include "PID.h"
#include "utils.h"

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
    // [this] remembers the current Controller instance, called as capture clause. 
    track_client->async_send_request(request,
        [this](rclcpp::Client<track_srv::srv::ReturnTrack>::SharedFuture future) {
            auto response = future.get();
            
            // Handle the response data 
            this->latest_track_center = response->points; 
            RCLCPP_INFO(this->get_logger(), "Track received. Controller ready for live profiling.");
            
            // Compute the static profile once, right after receiving the track
            utils::VehicleParams config;
            this->v_profile = utils::computeSmoothVel(this->latest_track_center, config, /*current_speed=*/0.0,
                                                    this->v_corner, this->v_accln, this->v_brake);

            utils::saveProfileToCSV(this->v_profile, this->v_accln, this->v_brake, this->v_corner,
                                    "velocity_profile.csv");

            RCLCPP_INFO(this->get_logger(), "CSV Saved");
            
            // Velocity setpoint calculation
            this->s = utils::getCummulativeS(latest_track_center);

            // Total loop length = distance to last point + distance from last point back to P0
            double last_segment = std::hypot(
                latest_track_center.front().x - latest_track_center.back().x,
                latest_track_center.front().y - latest_track_center.back().y
            );
            this->track_length = s.back() + last_segment;
                
            this->has_received_track = true;
        }); 

}

void Controller::controllerCallback(const lfs_msgs::msg::BikeState::SharedPtr msg){

    // Check if the controller callback has received the track center points and then request for it 
    if (!has_received_track) {
        RCLCPP_WARN(this->get_logger(), "Waiting for first track center points...");
        return;
    }

    double current_speed = static_cast<double>(msg->x_dot);
    double current_s = static_cast<double>(msg->s); 

    // Setpoint generation 

    double current_progress = std::fmod(current_s, track_length);  // Helps to give the spline progression even after multiple laps

    // front, back is for values. begin and end gives iterators (memory pointers)
    auto it = std::upper_bound(s.begin(), s.end(), current_progress); // Search the sorted container s (from [first element] to (last)) 
    //and return an iterator pointing to the first element that is strictly greater than current_progress.

    size_t idx_next = 0;
    size_t idx_prev = 0;
    // this idx basically gives the index of 0 to 98 points that is stored in cummulative S in utils

    if (it == s.end()) { //
        // progress between the very last point and the first point (P0)
        idx_prev = s.size() - 1;
        idx_next = 0;
    } else if (it == s.begin()) {
        // progress is exactly at or before 0.0
        idx_prev = 0;
        idx_next = 1;
    } else {
        // progress falls cleanly between two track points
        idx_next = std::distance(s.begin(), it); // index distance (ex: 0 to 5 = 5 steps)
        idx_prev = idx_next - 1;
    } 

    // Linear interpolation for smooth velocity profile
    double s_prev = s[idx_prev];
    double s_next;

    if (idx_next == 0){
        // after 1 lap 
        s_next = track_length;
    }
    else{
        s_next = s[idx_next];
    }

    double t = (current_progress - s_prev) / (s_next - s_prev); 

    // v_profile vectors of prev and next indexes 
    double v_prev = v_profile[idx_prev];
    double v_next = v_profile[idx_next];

    double target_velocity = v_prev + t*(v_next - v_prev);
    
    // PID Controller
    double u; 

    u = pid_1.calculateOutput(current_speed, target_velocity); 

    std_msgs::msg::Float64 throttle_msg; 
    throttle_msg.data = u; 

    throttle_publisher->publish(throttle_msg); 

    // Debug
    RCLCPP_INFO(this->get_logger(), 
            "--- VERIFICATION --- Lap Progress: %.2fm | Match Index: [ %zu ] | Interp t: %.2f | Target V: %.2f m/s | Actual V: %.2f m/s", 
            current_progress, idx_prev, t, target_velocity, current_speed);    
}